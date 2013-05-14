#
# Makefile
#
# Copyright (C) 2013, Galois, Inc.
# All Rights Reserved.
#

# Set this to the installation directory for V-REP.
VREP_PREFIX ?= $(HOME)/Downloads/V-REP_PRO_EDU_V3_0_3_64_Linux

CXX         ?= g++
INCLUDES    := -I. -I$(VREP_PREFIX)/programming/include
DEFINES     := -DPIC -D__linux
CXXFLAGS    := -std=c++11 -fPIC -Wall -g $(INCLUDES) $(DEFINES)

LIB         := libv_repExtQuadcopter.so
SOURCES     := Quadcopter.cpp           \
               SimGPS.cpp               \
               v_repExtQuadcopter.cpp   \
               $(VREP_PREFIX)/programming/common/v_repLib.cpp
LIBS        := -lGeographic

O           := obj/
OBJECTS     := $(patsubst %.cpp,$(O)%.o,$(SOURCES))
DEPS        := $(patsubst %.cpp,$(O)%.d,$(SOURCES))

all: $(LIB)

$(LIB): $(OBJECTS)
	@echo "LINK $@"
	@$(CXX) $(CXXFLAGS) -shared -o $@ $(OBJECTS) $(LIBS)

$(O)%.o: %.cpp
	@mkdir -p $(dir $@)
	@echo "CXX $(notdir $<)"
	@$(CXX) $(CXXFLAGS) -MMD -c -o $@ $<

.PHONY: clean
clean:
	rm -f $(LIB) $(OBJECTS) $(DEPS)

-include $(DEPS)

