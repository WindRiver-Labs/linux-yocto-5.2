// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2018, Intel Corporation. */

#include "ice_common.h"

/**
 * ice_aq_read_nvm
 * @hw: pointer to the HW struct
 * @module_typeid: module pointer location in words from the NVM beginning
 * @offset: byte offset from the module beginning
 * @length: length of the section to be read (in bytes from the offset)
 * @data: command buffer (size [bytes] = length)
 * @last_command: tells if this is the last command in a series
 * @read_shadow_ram: tell if this is a shadow RAM read
 * @cd: pointer to command details structure or NULL
 *
 * Read the NVM using the admin queue commands (0x0701)
 */
static enum ice_status
ice_aq_read_nvm(struct ice_hw *hw, u16 module_typeid, u32 offset, u16 length,
		void *data, bool last_command, bool read_shadow_ram,
		struct ice_sq_cd *cd)
{
	struct ice_aq_desc desc;
	struct ice_aqc_nvm *cmd;

	cmd = &desc.params.nvm;

	/* In offset the highest byte must be zeroed. */
	if (offset & 0xFF000000)
		return ICE_ERR_PARAM;

	ice_fill_dflt_direct_cmd_desc(&desc, ice_aqc_opc_nvm_read);

	if (!read_shadow_ram && module_typeid == ICE_AQC_NVM_START_POINT)
		cmd->cmd_flags |= ICE_AQC_NVM_FLASH_ONLY;

	/* If this is the last command in a series, set the proper flag. */
	if (last_command)
		cmd->cmd_flags |= ICE_AQC_NVM_LAST_CMD;
	cmd->module_typeid = cpu_to_le16(module_typeid);
	cmd->offset_low = cpu_to_le16(offset & 0xFFFF);
	cmd->offset_high = (offset >> 16) & 0xFF;
	cmd->length = cpu_to_le16(length);

	return ice_aq_send_cmd(hw, &desc, data, length, cd);
}

/**
 * ice_read_flat_nvm - Read portion of NVM by flat offset
 * @hw: pointer to the HW struct
 * @offset: offset from beginning of NVM
 * @length: (in) number of bytes to read; (out) number of bytes actually read
 * @data: buffer to return data in (sized to fit the specified length)
 * @read_shadow_ram: if true, read from shadow RAM instead of NVM
 *
 * Reads a portion of the NVM, as a flat memory space. This function correctly
 * breaks read requests across Shadow RAM sectors and ensures that no single
 * read request exceeds the maximum 4Kb read for a single AdminQ command.
 *
 * Returns a status code on failure. Note that the data pointer may be
 * partially updated if some reads succeed before a failure.
 */
enum ice_status
ice_read_flat_nvm(struct ice_hw *hw, u32 offset, u32 *length, u8 *data,
		  bool read_shadow_ram)
{
	enum ice_status status;
	u32 inlen = *length;
	u32 bytes_read = 0;
	bool last_cmd;

	*length = 0;

	/* Verify the length of the read if this is for the Shadow RAM */
	if (read_shadow_ram && ((offset + inlen) > (hw->nvm.sr_words * 2u))) {
		ice_debug(hw, ICE_DBG_NVM,
			  "NVM error: requested offset is beyond Shadow RAM limit\n");
		return ICE_ERR_PARAM;
	}

	do {
		u32 read_size, sector_offset;

		/* ice_aq_read_nvm cannot read more than 4Kb at a time.
		 * Additionally, a read from the Shadow RAM may not cross over
		 * a sector boundary. Conveniently, the sector size is also
		 * 4Kb.
		 */
		sector_offset = offset % ICE_AQ_MAX_BUF_LEN;
		read_size = min_t(u32, ICE_AQ_MAX_BUF_LEN - sector_offset,
				  inlen - bytes_read);

		last_cmd = !(bytes_read + read_size < inlen);

		status = ice_aq_read_nvm(hw, ICE_AQC_NVM_START_POINT,
					 offset, read_size,
					 data + bytes_read, last_cmd,
					 read_shadow_ram, NULL);
		if (status)
			break;

		bytes_read += read_size;
		offset += read_size;
	} while (!last_cmd);

	*length = bytes_read;
	return status;
}

/**
 * ice_check_sr_access_params - verify params for Shadow RAM R/W operations.
 * @hw: pointer to the HW structure
 * @offset: offset in words from module start
 * @words: number of words to access
 */
static enum ice_status
ice_check_sr_access_params(struct ice_hw *hw, u32 offset, u16 words)
{
	if ((offset + words) > hw->nvm.sr_words) {
		ice_debug(hw, ICE_DBG_NVM,
			  "NVM error: offset beyond SR lmt.\n");
		return ICE_ERR_PARAM;
	}

	if (words > ICE_SR_SECTOR_SIZE_IN_WORDS) {
		/* We can access only up to 4KB (one sector), in one AQ write */
		ice_debug(hw, ICE_DBG_NVM,
			  "NVM error: tried to access %d words, limit is %d.\n",
			  words, ICE_SR_SECTOR_SIZE_IN_WORDS);
		return ICE_ERR_PARAM;
	}

	if (((offset + (words - 1)) / ICE_SR_SECTOR_SIZE_IN_WORDS) !=
	    (offset / ICE_SR_SECTOR_SIZE_IN_WORDS)) {
		/* A single access cannot spread over two sectors */
		ice_debug(hw, ICE_DBG_NVM,
			  "NVM error: cannot spread over two sectors.\n");
		return ICE_ERR_PARAM;
	}

	return 0;
}

/**
 * ice_read_sr_aq - Read Shadow RAM.
 * @hw: pointer to the HW structure
 * @offset: offset in words from module start
 * @words: number of words to read
 * @data: storage for the words read from Shadow RAM (Little Endian)
 * @last_command: tells the AdminQ that this is the last command
 *
 * Reads 16-bit Little Endian word buffers from the Shadow RAM using the admin
 * command.
 */
static enum ice_status
ice_read_sr_aq(struct ice_hw *hw, u32 offset, u16 words, __le16 *data,
	       bool last_command)
{
	enum ice_status status;

	status = ice_check_sr_access_params(hw, offset, words);

	/* values in "offset" and "words" parameters are sized as words
	 * (16 bits) but ice_aq_read_nvm expects these values in bytes.
	 * So do this conversion while calling ice_aq_read_nvm.
	 */
	if (!status)
		status = ice_aq_read_nvm(hw, 0, 2 * offset, 2 * words, data,
					 last_command, true, NULL);

	return status;
}

/**
 * ice_read_sr_word_aq - Reads Shadow RAM via AQ
 * @hw: pointer to the HW structure
 * @offset: offset of the Shadow RAM word to read (0x000000 - 0x001FFF)
 * @data: word read from the Shadow RAM
 *
 * Reads one 16 bit word from the Shadow RAM using ice_read_flat_nvm.
 */
static enum ice_status
ice_read_sr_word_aq(struct ice_hw *hw, u16 offset, u16 *data)
{
	u32 bytes = sizeof(u16);
	enum ice_status status;
	__le16 data_local;

	/* Note that ice_read_flat_nvm takes into account the 4Kb AdminQ and
	 * Shadow RAM sector restrictions necessary when reading from the NVM.
	 */
	status = ice_read_flat_nvm(hw, offset * sizeof(u16), &bytes,
				   (u8 *)&data_local, true);
	if (status)
		return status;

	*data = le16_to_cpu(data_local);
	return 0;
}

/**
 * ice_read_sr_buf_aq - Reads Shadow RAM buf via AQ
 * @hw: pointer to the HW structure
 * @offset: offset of the Shadow RAM word to read (0x000000 - 0x001FFF)
 * @words: (in) number of words to read; (out) number of words actually read
 * @data: words read from the Shadow RAM
 *
 * Reads 16 bit words (data buf) from the SR using the ice_read_sr_aq
 * method. Ownership of the NVM is taken before reading the buffer and later
 * released.
 */
static enum ice_status
ice_read_sr_buf_aq(struct ice_hw *hw, u16 offset, u16 *words, u16 *data)
{
	enum ice_status status;
	bool last_cmd = false;
	u16 words_read = 0;
	u16 i = 0;

	do {
		u16 read_size, off_w;

		/* Calculate number of bytes we should read in this step.
		 * It's not allowed to read more than one page at a time or
		 * to cross page boundaries.
		 */
		off_w = offset % ICE_SR_SECTOR_SIZE_IN_WORDS;
		read_size = off_w ?
			min_t(u16, *words,
			      (ICE_SR_SECTOR_SIZE_IN_WORDS - off_w)) :
			min_t(u16, (*words - words_read),
			      ICE_SR_SECTOR_SIZE_IN_WORDS);

		/* Check if this is last command, if so set proper flag */
		if ((words_read + read_size) >= *words)
			last_cmd = true;

		status = ice_read_sr_aq(hw, offset, read_size,
					data + words_read, last_cmd);
		if (status)
			goto read_nvm_buf_aq_exit;

		/* Increment counter for words already read and move offset to
		 * new read location
		 */
		words_read += read_size;
		offset += read_size;
	} while (words_read < *words);

	for (i = 0; i < *words; i++)
		data[i] = le16_to_cpu(((__force __le16 *)data)[i]);

read_nvm_buf_aq_exit:
	*words = words_read;
	return status;
}

/**
 * ice_acquire_nvm - Generic request for acquiring the NVM ownership
 * @hw: pointer to the HW structure
 * @access: NVM access type (read or write)
 *
 * This function will request NVM ownership.
 */
static enum ice_status
ice_acquire_nvm(struct ice_hw *hw, enum ice_aq_res_access_type access)
{
	if (hw->nvm.blank_nvm_mode)
		return 0;

	return ice_acquire_res(hw, ICE_NVM_RES_ID, access, ICE_NVM_TIMEOUT);
}

/**
 * ice_release_nvm - Generic request for releasing the NVM ownership
 * @hw: pointer to the HW structure
 *
 * This function will release NVM ownership.
 */
static void ice_release_nvm(struct ice_hw *hw)
{
	if (hw->nvm.blank_nvm_mode)
		return;

	ice_release_res(hw, ICE_NVM_RES_ID);
}

/**
 * ice_read_sr_word - Reads Shadow RAM word and acquire NVM if necessary
 * @hw: pointer to the HW structure
 * @offset: offset of the Shadow RAM word to read (0x000000 - 0x001FFF)
 * @data: word read from the Shadow RAM
 *
 * Reads one 16 bit word from the Shadow RAM using the ice_read_sr_word_aq.
 */
enum ice_status ice_read_sr_word(struct ice_hw *hw, u16 offset, u16 *data)
{
	enum ice_status status;

	status = ice_acquire_nvm(hw, ICE_RES_READ);
	if (!status) {
		status = ice_read_sr_word_aq(hw, offset, data);
		ice_release_nvm(hw);
	}

	return status;
}

/**
 * ice_get_orom_ver_info - Read Option ROM version information
 * @hw: pointer to the HW struct
 *
 * Read the Combo Image version data from the Boot Configuration TLV and fill
 * in the option ROM version data.
 */
static enum ice_status ice_get_orom_ver_info(struct ice_hw *hw)
{
	u16 combo_hi, combo_lo, boot_cfg_tlv, boot_cfg_tlv_len;
	struct ice_orom_info *orom = &hw->nvm.orom;
	enum ice_status status;
	u32 combo_ver;

	status = ice_get_pfa_module_tlv(hw, &boot_cfg_tlv, &boot_cfg_tlv_len,
					ICE_SR_BOOT_CFG_PTR);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT,
			  "Failed to read Boot Configuration Block TLV.\n");
		return status;
	}

	/* Boot Configuration Block must have length at least 2 words
	 * (Combo Image Version High and Combo Image Version Low)
	 */
	if (boot_cfg_tlv_len < 2) {
		ice_debug(hw, ICE_DBG_INIT,
			  "Invalid Boot Configuration Block TLV size.\n");
		return ICE_ERR_INVAL_SIZE;
	}

	status = ice_read_sr_word(hw, (boot_cfg_tlv + ICE_NVM_OROM_VER_OFF),
				  &combo_hi);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT, "Failed to read OROM_VER hi.\n");
		return status;
	}

	status = ice_read_sr_word(hw, (boot_cfg_tlv + ICE_NVM_OROM_VER_OFF + 1),
				  &combo_lo);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT, "Failed to read OROM_VER lo.\n");
		return status;
	}

	combo_ver = ((u32)combo_hi << 16) | combo_lo;

	orom->major = (u8)((combo_ver & ICE_OROM_VER_MASK) >>
			   ICE_OROM_VER_SHIFT);
	orom->patch = (u8)(combo_ver & ICE_OROM_VER_PATCH_MASK);
	orom->build = (u16)((combo_ver & ICE_OROM_VER_BUILD_MASK) >>
			    ICE_OROM_VER_BUILD_SHIFT);

	return 0;
}

/**
 * ice_init_nvm - initializes NVM setting
 * @hw: pointer to the HW struct
 *
 * This function reads and populates NVM settings such as Shadow RAM size,
 * max_timeout, and blank_nvm_mode
 */
enum ice_status ice_init_nvm(struct ice_hw *hw)
{
	struct ice_nvm_info *nvm = &hw->nvm;
	u16 eetrack_lo, eetrack_hi, ver;
	enum ice_status status;
	u32 fla, gens_stat;
	u8 sr_size;

	/* The SR size is stored regardless of the NVM programming mode
	 * as the blank mode may be used in the factory line.
	 */
	gens_stat = rd32(hw, GLNVM_GENS);
	sr_size = (gens_stat & GLNVM_GENS_SR_SIZE_M) >> GLNVM_GENS_SR_SIZE_S;

	/* Switching to words (sr_size contains power of 2) */
	nvm->sr_words = BIT(sr_size) * ICE_SR_WORDS_IN_1KB;

	/* Check if we are in the normal or blank NVM programming mode */
	fla = rd32(hw, GLNVM_FLA);
	if (fla & GLNVM_FLA_LOCKED_M) { /* Normal programming mode */
		nvm->blank_nvm_mode = false;
	} else {
		/* Blank programming mode */
		nvm->blank_nvm_mode = true;
		ice_debug(hw, ICE_DBG_NVM,
			  "NVM init error: unsupported blank mode.\n");
		return ICE_ERR_NVM_BLANK_MODE;
	}

	status = ice_read_sr_word(hw, ICE_SR_NVM_DEV_STARTER_VER, &ver);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT,
			  "Failed to read DEV starter version.\n");
		return status;
	}
	nvm->major_ver = (ver & ICE_NVM_VER_HI_MASK) >> ICE_NVM_VER_HI_SHIFT;
	nvm->minor_ver = (ver & ICE_NVM_VER_LO_MASK) >> ICE_NVM_VER_LO_SHIFT;

	status = ice_read_sr_word(hw, ICE_SR_NVM_EETRACK_LO, &eetrack_lo);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT, "Failed to read EETRACK lo.\n");
		return status;
	}
	status = ice_read_sr_word(hw, ICE_SR_NVM_EETRACK_HI, &eetrack_hi);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT, "Failed to read EETRACK hi.\n");
		return status;
	}

	nvm->eetrack = (eetrack_hi << 16) | eetrack_lo;

	switch (hw->device_id) {
	/* the following devices do not have boot_cfg_tlv yet */
	case ICE_DEV_ID_E823C_BACKPLANE:
	case ICE_DEV_ID_E823C_QSFP:
	case ICE_DEV_ID_E823C_SFP:
	case ICE_DEV_ID_E823C_10G_BASE_T:
	case ICE_DEV_ID_E823C_SGMII:
	case ICE_DEV_ID_E822C_BACKPLANE:
	case ICE_DEV_ID_E822C_QSFP:
	case ICE_DEV_ID_E822C_10G_BASE_T:
	case ICE_DEV_ID_E822C_SGMII:
	case ICE_DEV_ID_E822C_SFP:
	case ICE_DEV_ID_E822L_BACKPLANE:
	case ICE_DEV_ID_E822L_SFP:
	case ICE_DEV_ID_E822L_10G_BASE_T:
	case ICE_DEV_ID_E822L_SGMII:
	case ICE_DEV_ID_E823L_BACKPLANE:
	case ICE_DEV_ID_E823L_SFP:
	case ICE_DEV_ID_E823L_10G_BASE_T:
	case ICE_DEV_ID_E823L_1GBE:
	case ICE_DEV_ID_E823L_QSFP:
		return status;
	default:
		break;
	}

	status = ice_get_orom_ver_info(hw);
	if (status) {
		ice_debug(hw, ICE_DBG_INIT, "Failed to read Option ROM info.\n");
		return status;
	}

	return 0;
}

/**
 * ice_read_sr_buf - Reads Shadow RAM buf and acquire lock if necessary
 * @hw: pointer to the HW structure
 * @offset: offset of the Shadow RAM word to read (0x000000 - 0x001FFF)
 * @words: (in) number of words to read; (out) number of words actually read
 * @data: words read from the Shadow RAM
 *
 * Reads 16 bit words (data buf) from the SR using the ice_read_nvm_buf_aq
 * method. The buf read is preceded by the NVM ownership take
 * and followed by the release.
 */
enum ice_status
ice_read_sr_buf(struct ice_hw *hw, u16 offset, u16 *words, u16 *data)
{
	enum ice_status status;

	status = ice_acquire_nvm(hw, ICE_RES_READ);
	if (!status) {
		status = ice_read_sr_buf_aq(hw, offset, words, data);
		ice_release_nvm(hw);
	}

	return status;
}

/**
 * ice_nvm_validate_checksum
 * @hw: pointer to the HW struct
 *
 * Verify NVM PFA checksum validity (0x0706)
 */
enum ice_status ice_nvm_validate_checksum(struct ice_hw *hw)
{
	struct ice_aqc_nvm_checksum *cmd;
	struct ice_aq_desc desc;
	enum ice_status status;

	status = ice_acquire_nvm(hw, ICE_RES_READ);
	if (status)
		return status;

	cmd = &desc.params.nvm_checksum;

	ice_fill_dflt_direct_cmd_desc(&desc, ice_aqc_opc_nvm_checksum);
	cmd->flags = ICE_AQC_NVM_CHECKSUM_VERIFY;

	status = ice_aq_send_cmd(hw, &desc, NULL, 0, NULL);
	ice_release_nvm(hw);

	if (!status)
		if (le16_to_cpu(cmd->checksum) != ICE_AQC_NVM_CHECKSUM_CORRECT)
			status = ICE_ERR_NVM_CHECKSUM;

	return status;
}
