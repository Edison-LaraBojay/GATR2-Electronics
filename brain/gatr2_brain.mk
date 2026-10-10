# gatr2_brain.mk
# GATR2 Brain libraries for a PROS project. Compiles the spectaGATR publisher
# with its wire format in place from
# this repository into $(BINDIR)/gatr2/. Include it twice from the project
# Makefile, before common.mk (sources, objects, include paths) and after it
# (compile rules):
#
#   GATR2_ROOT ?= ../..
#   GATR2_BRAIN_LIBS := spectagatr
#   include $(GATR2_ROOT)/brain/gatr2_brain.mk
#   -include ./common.mk
#   include $(GATR2_ROOT)/brain/gatr2_brain.mk
#
# The spectaGATR pose publisher alone: GATR2_BRAIN_LIBS := spectagatr

ifndef GATR2_BRAIN_SETUP
GATR2_BRAIN_SETUP := 1

# Repository root as a path relative to the PROS project. Drive letter paths
# break the include flags in the toolchain shell.
GATR2_ROOT ?= ../..

# Libraries to build. The pose publisher is independent of the pose source.
GATR2_BRAIN_LIBS ?= spectagatr

# Sources per library, listed explicitly.
GATR2_SRC_spectagatr := \
	$(GATR2_ROOT)/brain/spectaGATR/src/publisher.cpp \
	$(GATR2_ROOT)/brain/spectaGATR/pros/pros_publisher.cpp \
	$(GATR2_ROOT)/spectaGATR/protocol/src/pose_line.cpp \
	$(GATR2_ROOT)/spectaGATR/protocol/src/units.cpp

GATR2_SRC := $(foreach lib,$(GATR2_BRAIN_LIBS),$(GATR2_SRC_$(lib)))
GATR2_OBJ := $(patsubst $(GATR2_ROOT)/%,$(BINDIR)/gatr2/%.o,$(GATR2_SRC))

EXTRA_INCDIR += \
	$(GATR2_ROOT)/brain/spectaGATR/include \
	$(GATR2_ROOT)/spectaGATR/protocol/include \
	$(GATR2_ROOT)
ELF_DEPS += $(GATR2_OBJ)

else

GATR2_WARNFLAGS ?= -Wall -Wextra

# Own compile rule. common.mk's dependency steps (DEPFLAGS, mv of the .Td file)
# assume sources under src/ and would write over the library sources.
# Dependency files go next to the objects.
define gatr2_src_rule
$(patsubst $(GATR2_ROOT)/%,$(BINDIR)/gatr2/%.o,$1): $1
	$(VV)mkdir -p $$(dir $$@)
	$$(call test_output_2,Compiled $$< ,$(CXX) -c $(INCLUDE) $(CXXFLAGS) $(EXTRA_CXXFLAGS) $(GATR2_WARNFLAGS) -MMD -MP -MF $$(@:.o=.d) -o $$@ $$<,$(OK_STRING))
endef
$(foreach src,$(GATR2_SRC),$(eval $(call gatr2_src_rule,$(src))))

-include $(GATR2_OBJ:.o=.d)

endif
