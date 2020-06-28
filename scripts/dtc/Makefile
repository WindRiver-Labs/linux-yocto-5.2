# SPDX-License-Identifier: GPL-2.0
# scripts/dtc makefile

hostprogs-$(CONFIG_DTC) := dtc
always		:= $(hostprogs-y)

dtc-objs	:= dtc.o flattree.o fstree.o data.o livetree.o treesource.o \
		   srcpos.o checks.o util.o
dtc-objs	+= dtc-lexer.lex.o dtc-parser.tab.o

# Source files need to get at the userspace version of libfdt_env.h to compile
HOST_EXTRACFLAGS := -I $(srctree)/$(src)/libfdt

_yaml_libs = $(shell pkg-config --libs yaml-0.1 2>/dev/null)
ifeq ($(call hostcc-header,yaml.h)$(call hostcc-symbol,yaml_get_version,\
				    $(_yaml_libs)),yy)
dtc-objs	+= yamltree.o
HOSTLDLIBS_dtc	:= $(_yaml_libs)
else
ifneq ($(CHECK_DTBS),)
$(error dtc needs libyaml for DT schema validation support. \
	Install the necessary libyaml development package.)
endif
HOST_EXTRACFLAGS += -DNO_YAML
endif

# Generated files need one more search path to include headers in source tree
HOSTCFLAGS_dtc-lexer.lex.o := -I $(srctree)/$(src)
HOSTCFLAGS_dtc-parser.tab.o := -I $(srctree)/$(src)

# dependencies on generated files need to be listed explicitly
$(obj)/dtc-lexer.lex.o: $(obj)/dtc-parser.tab.h
