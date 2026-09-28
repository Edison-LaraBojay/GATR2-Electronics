################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=-Wall -Wextra
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=-D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING

# The kernel template defaults (gnu23, gnu++26) are rejected by arm gcc 13.
# Kernel headers need C++20.
C_STANDARD:=gnu17
CXX_STANDARD:=gnu++20

# GATR2 Brain libraries, compiled in place from this repository by
# brain/gatr2_brain.mk. This application uses all three.
GATR2_ROOT ?= ../..
GATR2_BRAIN_LIBS := investigatr communigatr actugatr

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:=

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

# Kernel files (firmware/, include/pros/) are not in git. Restore them offline
# from the local PROS template cache, in this folder:
#   pros c apply kernel@4.2.2 --force-apply --no-download
#   git checkout -- .gitignore      (the CLI replaces it with the template copy)
ifneq ($(words $(wildcard $(FWDIR)/libpros.a $(INCDIR)/pros/rtos.hpp)),2)
$(error PROS kernel files missing. In brain/operaGATR run: pros c apply kernel@4.2.2 --force-apply --no-download then git checkout -- .gitignore)
endif

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
include $(GATR2_ROOT)/brain/gatr2_brain.mk
-include ./common.mk
include $(GATR2_ROOT)/brain/gatr2_brain.mk
