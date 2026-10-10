# gatr2_brain.mk
# GATR2 Brain libraries for a PROS project. Compiles investiGATR,
# actuGATR and the spectaGATR publisher with its wire format in place from
# this repository into $(BINDIR)/gatr2/. Include it twice from the project
# Makefile, before common.mk (sources, objects, include paths) and after it
# (compile rules):
#
#   GATR2_ROOT ?= ../..
#   GATR2_BRAIN_LIBS := investigatr actugatr
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

# Libraries to build. actugatr needs investigatr; spectagatr needs neither.
GATR2_BRAIN_LIBS ?= investigatr actugatr

# Sources per library, listed explicitly.
GATR2_SRC_investigatr := \
	$(GATR2_ROOT)/brain/investiGATR/src/geometry.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/field.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/state_source.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/reference.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/motion_model.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/path.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/collision.cpp \
	$(GATR2_ROOT)/brain/investiGATR/src/planner.cpp

GATR2_SRC_actugatr := \
	$(GATR2_ROOT)/brain/actuGATR/src/chassis.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/pid.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/kinematics.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/drivetrain.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/drive.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/follower.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/motion.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/drive_owner.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/drive_requests.cpp \
	$(GATR2_ROOT)/brain/actuGATR/src/ports.cpp \
	$(GATR2_ROOT)/brain/actuGATR/pros/pros_motor_output.cpp \
	$(GATR2_ROOT)/brain/actuGATR/pros/pros_drive.cpp

GATR2_SRC_spectagatr := \
	$(GATR2_ROOT)/brain/spectaGATR/src/publisher.cpp \
	$(GATR2_ROOT)/brain/spectaGATR/pros/pros_publisher.cpp \
	$(GATR2_ROOT)/spectaGATR/protocol/src/pose_line.cpp \
	$(GATR2_ROOT)/spectaGATR/protocol/src/units.cpp

GATR2_SRC := $(foreach lib,$(GATR2_BRAIN_LIBS),$(GATR2_SRC_$(lib)))
GATR2_OBJ := $(patsubst $(GATR2_ROOT)/%,$(BINDIR)/gatr2/%.o,$(GATR2_SRC))

EXTRA_INCDIR += \
	$(GATR2_ROOT)/brain/investiGATR/include \
	$(GATR2_ROOT)/brain/actuGATR/include \
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
