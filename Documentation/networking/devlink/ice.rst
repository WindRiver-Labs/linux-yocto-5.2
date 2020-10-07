.. SPDX-License-Identifier: GPL-2.0

===================
ice devlink support
===================

This document describes the devlink features implemented by the ``ice``
device driver.

Info versions
=============

The ``ice`` driver reports the following versions

.. list-table:: devlink info versions implemented
    :widths: 5 5 5 90

    * - Name
      - Type
      - Example
      - Description
    * - ``board.id``
      - fixed
      - K65390-000
      - The Product Board Assembly (PBA) identifier of the board.
    * - ``fw.mgmt``
      - running
      - 2.1.7
      - 3-digit version number of the management firmware that controls the
        PHY, link, etc.
    * - ``fw.mgmt.api``
      - running
      - 1.5
      - 2-digit version number of the API exported over the AdminQ by the
        management firmware. Used by the driver to identify what commands
        are supported.
    * - ``fw.mgmt.build``
      - running
      - 0x305d955f
      - Unique identifier of the source for the management firmware.
    * - ``fw.undi``
      - running
      - 1.2581.0
      - Version of the Option ROM containing the UEFI driver. The version is
        reported in ``major.minor.patch`` format. The major version is
        incremented whenever a major breaking change occurs, or when the
        minor version would overflow. The minor version is incremented for
        non-breaking changes and reset to 1 when the major version is
        incremented. The patch version is normally 0 but is incremented when
        a fix is delivered as a patch against an older base Option ROM.
    * - ``fw.psid.api``
      - running
      - 0.80
      - Version defining the format of the flash contents.
    * - ``fw.bundle_id``
      - running
      - 0x80002ec0
      - Unique identifier of the firmware image file that was loaded onto
        the device. Also referred to as the EETRACK identifier of the NVM.
    * - ``fw.app.name``
      - running
      - ICE OS Default Package
      - The name of the DDP package that is active in the device. The DDP
        package is loaded by the driver during initialization. Each varation
        of DDP package shall have a unique name.
    * - ``fw.app``
      - running
      - 1.3.1.0
      - The version of the DDP package that is active in the device. Note
        that both the name (as reported by ``fw.app.name``) and version are
        required to uniquely identify the package.
    * - ``fw.app.bundle_id``
      - 0xc0000001
      - Unique identifier for the DDP package loaded in the device. Also
        referred to as the DDP Track ID. Can be used to uniquely identify
        the specific DDP package.
    * - ``fw.netlist``
      - running
      - 1.1.2000-6.7.0
      - The version of the netlist module. This module defines the device's
        Ethernet capabilities and default settings, and is used by the
        management firmware as part of managing link and device
        connectivity.
    * - ``fw.netlist.build``
      - running
      - 0xee16ced7
      - The first 4 bytes of the hash of the netlist module contents.
