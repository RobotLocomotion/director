# Default makefile distributed with pods version: 12.01.11

BUILD_SYSTEM:=$(OS)
ifeq ($(BUILD_SYSTEM),Windows_NT)
ifneq ($(CYGWIN),)# check if it's cygwin
BUILD_SYSTEM:=Cygwin
endif
else
BUILD_SYSTEM:=$(shell uname -s)
endif

# Figure out where to build the software.
#   Use BUILD_PREFIX if it was passed in.
#   If not, search up to four parent directories for a 'build' directory.
#   Otherwise, use ./build.
ifeq ($(BUILD_SYSTEM), Windows_NT)
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell (for %%x in (. .. ..\.. ..\..\.. ..\..\..\..) do ( if exist %cd%\%%x\build ( echo %cd%\%%x\build & exit ) )) & echo %cd%\build )
endif
# don't clean up and create build dir as I do in linux.  instead create it during configure.
else
ifeq "$(BUILD_PREFIX)" ""
BUILD_PREFIX:=$(shell for pfx in ./ .. ../.. ../../.. ../../../..; do d=`pwd`/$$pfx/build;\
               if [ -d $$d ]; then echo $$d; exit 0; fi; done; echo `pwd`/build)
endif
# create the build directory if needed, and normalize its path name
BUILD_PREFIX:=$(shell mkdir -p $(BUILD_PREFIX) && cd $(BUILD_PREFIX) && echo `pwd`)
endif

# Default to a release build.  If you want to enable debugging flags, run
# "make BUILD_TYPE=Debug"
ifeq "$(BUILD_TYPE)" ""
BUILD_TYPE="Release"
endif

all: pod-build/Makefile
	cmake --build pod-build --config $(BUILD_TYPE) --target install

pod-build/Makefile:
	$(MAKE) configure

.PHONY: configure
configure:
	@echo "BUILD_PREFIX: $(BUILD_PREFIX)"

# create the temporary build directory if needed
ifeq ($(BUILD_SYSTEM), Windows_NT)
	@if not exist $(BUILD_PREFIX) ( mkdir $(BUILD_PREFIX) )
	@if not exist pod-build ( mkdir pod-build )
else
	@mkdir -p pod-build
endif

# run CMake to generate and configure the build scripts
	@cd pod-build && cmake $(CMAKE_FLAGS) \
		   -DCMAKE_INSTALL_PREFIX=$(BUILD_PREFIX) \
		   -DCMAKE_BUILD_TYPE=$(BUILD_TYPE) \
		   -DCMAKE_PREFIX_PATH:PATH=$(BUILD_PREFIX) \
		   -DDRAKE_DIR:PATH=$(BUILD_PREFIX)/../drake \
		   -DUSE_DRC_MAPS:BOOL=ON \
		   -DUSE_DRC_PLANE_SEG:BOOL=ON \
		   ..

clean:
	-if [ -e pod-build/install_manifest.txt ]; then rm -f `cat pod-build/install_manifest.txt`; fi
	-if [ -d pod-build ]; then $(MAKE) -C pod-build clean; rm -rf pod-build; fi

# other (custom) targets are passed through to the cmake-generated Makefile
%::
	$(MAKE) -C pod-build $@

# Default to a less-verbose build.  If you want all the gory compiler output,
# run "make VERBOSE=1"
$(VERBOSE).SILENT:
