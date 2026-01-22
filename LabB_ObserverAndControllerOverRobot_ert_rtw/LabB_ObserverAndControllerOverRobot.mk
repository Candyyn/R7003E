###########################################################################
## Makefile generated for component 'LabB_ObserverAndControllerOverRobot'. 
## 
## Makefile     : LabB_ObserverAndControllerOverRobot.mk
## Generated on : Wed Jan 21 10:39:56 2026
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/LabB_ObserverAndControllerOverRobot.elf
## Product type : executable
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile

PRODUCT_NAME              = LabB_ObserverAndControllerOverRobot
MAKEFILE                  = LabB_ObserverAndControllerOverRobot.mk
MATLAB_ROOT               = /usr/local/MATLAB/R2025b
MATLAB_BIN                = /usr/local/MATLAB/R2025b/bin
MATLAB_ARCH_BIN           = $(MATLAB_BIN)/glnxa64
START_DIR                 = /home/emil/Skrivbord/r7003
SOLVER                    = 
SOLVER_OBJ                = 
CLASSIC_INTERFACE         = 0
TGT_FCN_LIB               = None
MODEL_HAS_DYNAMICALLY_LOADED_SFCNS = 0
RELATIVE_PATH_TO_ANCHOR   = ..
SLIB_PATH                 = /home/emil/Documents/MATLAB/R2025b/ArduinoStaticLibrary/ArduinoMega2560/FasterRuns
SSLIB_PATH                = /home/emil/Documents/MATLAB/R2025b/ArduinoStaticLibrary/ArduinoMega2560/FasterRuns
C_STANDARD_OPTS           = 
CPP_STANDARD_OPTS         = 

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

# Toolchain Name:          Arduino AVR
# Supported Version(s):    
# ToolchainInfo Version:   2025b
# Specification Revision:  1.0
# 
#-------------------------------------------
# Macros assumed to be defined elsewhere
#-------------------------------------------

# ARDUINO_ROOT
# ARDUINO_PORT
# ARDUINO_MCU
# ARDUINO_BAUD
# ARDUINO_PROTOCOL
# ARDUINO_F_CPU

#-----------
# MACROS
#-----------

PRODUCT_HEX      = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).hex
PRODUCT_BIN      = $(RELATIVE_PATH_TO_ANCHOR)/$(PRODUCT_NAME).eep
ARDUINO_TOOLS    = $(ARDUINO_AVR_ROOT)/tools/avr-gcc/$(AVR_GCC_LIB_VERSION)/bin
ELF2EEP_OPTIONS  = -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0
DOWNLOAD_ARGS    =  >tmp.trash 2>&1 -P$(ARDUINO_PORT) -V -q -q -q -q -F -C$(ARDUINO_ROOT)/hardware/tools/avr/etc/avrdude.conf -p$(ARDUINO_MCU) -c$(ARDUINO_PROTOCOL) -b$(ARDUINO_BAUD) -D -Uflash:w:

TOOLCHAIN_SRCS = 
TOOLCHAIN_INCS = 
TOOLCHAIN_LIBS = -lm -lcore

#------------------------
# BUILD TOOL COMMANDS
#------------------------

# Assembler: Arduino AVR Assembler
AS_PATH = $(ARDUINO_TOOLS)
AS = "$(AS_PATH)/avr-gcc"

# C Compiler: Arduino AVR C Compiler
CC_PATH = $(ARDUINO_TOOLS)
CC = "$(CC_PATH)/avr-gcc"

# Linker: Arduino AVR Linker
LD_PATH = $(ARDUINO_TOOLS)
LD = "$(LD_PATH)/avr-gcc"

# C++ Compiler: Arduino AVR C++ Compiler
CPP_PATH = $(ARDUINO_TOOLS)
CPP = "$(CPP_PATH)/avr-g++"

# C++ Linker: Arduino AVR C++ Linker
CPP_LD_PATH = $(ARDUINO_TOOLS)
CPP_LD = "$(CPP_LD_PATH)/avr-gcc"

# Archiver: Arduino AVR Archiver
AR_PATH = $(ARDUINO_TOOLS)
AR = "$(AR_PATH)/avr-ar"

# MEX Tool: MEX Tool
MEX_PATH = $(MATLAB_ARCH_BIN)
MEX = "$(MEX_PATH)/mex"

# Binary Converter: Binary Converter
OBJCOPY_PATH = $(ARDUINO_TOOLS)
OBJCOPY = "$(OBJCOPY_PATH)/avr-objcopy"

# Hex Converter: Hex Converter
OBJCOPY_PATH = $(ARDUINO_TOOLS)
OBJCOPY = "$(OBJCOPY_PATH)/avr-objcopy"

# Download: Download
DOWNLOAD_PATH = $(ARDUINO_TOOLS)
DOWNLOAD = "$(DOWNLOAD_PATH)/avrdude"

# Execute: Execute
EXECUTE = $(PRODUCT)

# Builder: Make Tool
MAKE_PATH = %MATLAB%/bin/glnxa64
MAKE = "$(MAKE_PATH)/gmake"


#-------------------------
# Directives/Utilities
#-------------------------

ASDEBUG             = -g
AS_OUTPUT_FLAG      = -o
CDEBUG              = -g
C_OUTPUT_FLAG       = -o
LDDEBUG             = -g
OUTPUT_FLAG         = -o
CPPDEBUG            = -g
CPP_OUTPUT_FLAG     = -o
CPPLDDEBUG          = -g
OUTPUT_FLAG         = -o
ARDEBUG             =
STATICLIB_OUTPUT_FLAG =
MEX_DEBUG           = -g
RM                  =
ECHO                = echo
MV                  =
RUN                 =

#--------------------------------------
# "Faster Runs" Build Configuration
#--------------------------------------

ARFLAGS              = rcs
ASFLAGS              = -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -Wall \
                       -x assembler-with-cpp \
                       $(ASFLAGS_ADDITIONAL) \
                       $(DEFINES) \
                       $(INCLUDES) \
                       -c
OBJCOPYFLAGS_BIN     = $(ELF2EEP_OPTIONS) $(PRODUCT) $(PRODUCT_BIN)
CFLAGS               = -std=gnu11  \
                       -c \
                       -w \
                       -ffunction-sections \
                       -fdata-sections  \
                       -MMD \
                       -DARDUINO=10801  \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -Os
CPPFLAGS             = -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics  \
                       -c \
                       -w \
                       -ffunction-sections \
                       -fdata-sections  \
                       -MMD \
                       -DARDUINO=10801  \
                       -MMD -MP -MF"$(@:%.o=%.dep)" -MT"$@"  \
                       -Os
CPP_LDFLAGS          =  -w -Os -Wl,--gc-sections,--relax
CPP_SHAREDLIB_LDFLAGS  =
DOWNLOAD_FLAGS       = $(DOWNLOAD_ARGS)$(PRODUCT_HEX):i
EXECUTE_FLAGS        =
OBJCOPYFLAGS_HEX     = -O ihex -R .eeprom $(PRODUCT) $(PRODUCT_HEX)
LDFLAGS              =  -w -Os -Wl,--gc-sections,--relax
MEX_CPPFLAGS         =
MEX_CPPLDFLAGS       =
MEX_CFLAGS           =
MEX_LDFLAGS          =
MAKE_FLAGS           = -f $(MAKEFILE)
SHAREDLIB_LDFLAGS    =



###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/LabB_ObserverAndControllerOverRobot.elf
PRODUCT_TYPE = "executable"
BUILD_TYPE = "Top-Level Standalone Executable"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I$(START_DIR) -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/include -I/home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src -I/home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/include -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/utility -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility -I$(MATLAB_ROOT)/toolbox/target/shared/svd/common/include -I$(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert -I$(ARDUINO_AVR_ROOT)/hardware/avr/$(AVR_LIB_VERSION)/cores/arduino -I$(ARDUINO_AVR_ROOT)/hardware/avr/$(AVR_LIB_VERSION)/variants/mega -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/include -I/home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/include -I$(ARDUINO_AVR_ROOT)/tools/avr-gcc/$(AVR_GCC_LIB_VERSION)/avr/include

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_ = -D__MW_TARGET_USE_HARDWARE_RESOURCES_H__ -DMW_TIMERID=5 -DMW_PRESCALAR=64 -DMW_TIMERCOUNT=64286 -DMW_SCHEDULERCOUNTER=1 -DARDUINO_NUM_SERIAL_PORTS=4 -DARDUINO_SERIAL_RECEIVE_BUFFER_SIZE=64 -D_RTT_BAUDRATE_SERIAL0_=115200 -D_RTT_BAUDRATE_SERIAL1_=9600 -D_RTT_BAUDRATE_SERIAL2_=9600 -D_RTT_BAUDRATE_SERIAL3_=9600 -D_RTT_CONFIG_SERIAL0_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL1_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL2_=SERIAL_8N1 -D_RTT_CONFIG_SERIAL3_=SERIAL_8N1 -D_RTT_ANALOG_REF_=0 -D_RTT_OVERRUN_DIGITAL_PIN_=13 -DMW_NUM_PINS=70 -D_RTT_PWM_BLOCKS_ -D_ONBOARD_EEPROM_SIZE_=4096
DEFINES_BUILD_ARGS = -DCLASSIC_INTERFACE=0 -DALLOCATIONFCN=0 -DTERMFCN=1 -DONESTEPFCN=1 -DMAT_FILE=0 -DMULTI_INSTANCE_CODE=0 -DINTEGER_CODE=0 -DMT=0
DEFINES_CUSTOM = 
DEFINES_OPTS = -DTID01EQ=0
DEFINES_SKIPFORSIL = -DXCP_CUSTOM_PLATFORM -DEXIT_FAILURE=1 -DEXTMODE_DISABLEPRINTF -DEXTMODE_DISABLETESTING -DEXTMODE_DISABLE_ARGS_PROCESSING=1 -DSTACK_SIZE=64 -DRT
DEFINES_STANDARD = -DMODEL=LabB_ObserverAndControllerOverRobot -DNUMST=1 -DNCSTATES=0 -DHAVESTDIO -DMODEL_HAS_DYNAMICALLY_LOADED_SFCNS=0

DEFINES = $(DEFINES_) $(DEFINES_BUILD_ARGS) $(DEFINES_CUSTOM) $(DEFINES_OPTS) $(DEFINES_SKIPFORSIL) $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_SerialRead.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_SerialWrite.cpp /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/encoder_arduino.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/Wire.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/twi.c /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/I2Cdev.cpp /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/MPU6050.cpp /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/MPU6050wrapper.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_arduino_digitalio.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_PWM.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_PWMDriver.c /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/ArduinoPinHandleMap.cpp $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/LabB_ObserverAndControllerOverRobot.c $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/LabB_ObserverAndControllerOverRobot_data.c $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rtGetInf.c $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rtGetNaN.c $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rt_nonfinite.c /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoAVRScheduler.cpp

MAIN_SRC = $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/ert_main.c

ALL_SRCS = $(SRCS) $(MAIN_SRC)

###########################################################################
## OBJECTS
###########################################################################

OBJS = MW_SerialRead.o MW_SerialWrite.o encoder_arduino.o Wire.o twi.o I2Cdev.o MPU6050.o MPU6050wrapper.o MW_arduino_digitalio.o MW_PWM.o MW_PWMDriver.o ArduinoPinHandleMap.o LabB_ObserverAndControllerOverRobot.o LabB_ObserverAndControllerOverRobot_data.o rtGetInf.o rtGetNaN.o rt_nonfinite.o MW_ArduinoHWInit.o io_wrappers.o arduinoAVRScheduler.o

MAIN_OBJ = ert_main.o

ALL_OBJS = $(OBJS) $(MAIN_OBJ)

###########################################################################
## PREBUILT OBJECT FILES
###########################################################################

PREBUILT_OBJS = 

###########################################################################
## LIBRARIES
###########################################################################

LIBS = $(SSLIB_PATH)/MW_RebuildSrc_Core.o

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = 

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_SKIPFORSIL = -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -D_RUNONTARGETHARDWARE_BUILD_ -D_ROTH_MEGA2560_ -DARDUINO_NUM_SERIAL_PORTS=4
CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_SKIPFORSIL) $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_SKIPFORSIL = -mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -D_RUNONTARGETHARDWARE_BUILD_ -D_ROTH_MEGA2560_ -DARDUINO_NUM_SERIAL_PORTS=4
CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_SKIPFORSIL) $(CPPFLAGS_BASIC)

#---------------
# C++ Linker
#---------------

CPP_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_LDFLAGS_SKIPFORSIL = -mmcu=atmega2560 

CPP_LDFLAGS += $(CPP_LDFLAGS_) $(CPP_LDFLAGS_SKIPFORSIL)

#------------------------------
# C++ Shared Library Linker
#------------------------------

CPP_SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL = -mmcu=atmega2560 

CPP_SHAREDLIB_LDFLAGS += $(CPP_SHAREDLIB_LDFLAGS_) $(CPP_SHAREDLIB_LDFLAGS_SKIPFORSIL)

#-----------
# Linker
#-----------

LDFLAGS_ = -L"$(SLIB_PATH)"
LDFLAGS_SKIPFORSIL = -mmcu=atmega2560 

LDFLAGS += $(LDFLAGS_) $(LDFLAGS_SKIPFORSIL)

#--------------------------
# Shared Library Linker
#--------------------------

SHAREDLIB_LDFLAGS_ = -L"$(SLIB_PATH)"
SHAREDLIB_LDFLAGS_SKIPFORSIL = -mmcu=atmega2560 

SHAREDLIB_LDFLAGS += $(SHAREDLIB_LDFLAGS_) $(SHAREDLIB_LDFLAGS_SKIPFORSIL)

###########################################################################
## INLINED COMMANDS
###########################################################################


DERIVED_SRCS = $(subst .o,.dep,$(OBJS))

build:

%.dep:



-include arduino_macros.mk
-include codertarget_assembly_flags.mk
-include *.dep


###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build buildobj clean info prebuild postbuild download execute


all : build postbuild
	echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


buildobj : prebuild $(OBJS) $(PREBUILT_OBJS) $(LIBS)
	echo "### Successfully generated all binary outputs."


prebuild : 


postbuild : $(PRODUCT)
	echo "### Invoking postbuild tool "Binary Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_BIN)
	echo "### Done invoking postbuild tool."
	echo "### Invoking postbuild tool "Hex Converter" ..."
	$(OBJCOPY) $(OBJCOPYFLAGS_HEX)
	echo "### Done invoking postbuild tool."


download : postbuild
	echo "### Invoking postbuild tool "Download" ..."
	$(DOWNLOAD) $(DOWNLOAD_FLAGS)
	echo "### Done invoking postbuild tool."


execute : download
	echo "### Invoking postbuild tool "Execute" ..."
	$(EXECUTE) $(EXECUTE_FLAGS)
	echo "### Done invoking postbuild tool."


###########################################################################
## FINAL TARGET
###########################################################################

#-------------------------------------------
# Create a standalone executable            
#-------------------------------------------

$(PRODUCT) : $(OBJS) $(PREBUILT_OBJS) $(LIBS) $(MAIN_OBJ)
	echo "### Creating standalone executable "$(PRODUCT)" ..."
	$(CPP_LD) $(CPP_LDFLAGS) -o $(PRODUCT) $(OBJS) $(MAIN_OBJ) $(LIBS) $(SYSTEM_LIBS) $(TOOLCHAIN_LIBS)
	echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : %.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : %.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : %.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(RELATIVE_PATH_TO_ANCHOR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : ./tmp/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : ./tmp/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : ./tmp/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : ./tmp/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/rtw/c/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/simulink/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/simulink/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.c
	$(CC) $(CFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.s
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.S.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.S
	$(AS) $(ASFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/toolbox/simulink/blocks/src/%.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_SerialRead.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_SerialRead.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_SerialWrite.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_SerialWrite.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


encoder_arduino.o : /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/encoder_arduino.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


Wire.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/Wire.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


twi.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/3P.instrset/arduinoide.instrset/aCLI/data/packages/arduino/hardware/avr/1.8.3/libraries/Wire/src/utility/twi.c
	$(CC) $(CFLAGS) -o "$@" "$<"


I2Cdev.o : /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/I2Cdev.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MPU6050.o : /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/MPU6050.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MPU6050wrapper.o : /home/emil/MATLAB\ Add-Ons/Collections/Rensselaer\ Arduino\ Support\ Package\ Library\ \(RASPLib\)/RASPlib/src/MPU6050wrapper.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_arduino_digitalio.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_arduino_digitalio.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_PWM.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_PWM.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


MW_PWMDriver.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/MW_PWMDriver.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ArduinoPinHandleMap.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/ArduinoPinHandleMap.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


LabB_ObserverAndControllerOverRobot.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/LabB_ObserverAndControllerOverRobot.c
	$(CC) $(CFLAGS) -o "$@" "$<"


LabB_ObserverAndControllerOverRobot_data.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/LabB_ObserverAndControllerOverRobot_data.c
	$(CC) $(CFLAGS) -o "$@" "$<"


ert_main.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/ert_main.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetInf.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rtGetInf.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rtGetNaN.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rtGetNaN.c
	$(CC) $(CFLAGS) -o "$@" "$<"


rt_nonfinite.o : $(START_DIR)/LabB_ObserverAndControllerOverRobot_ert_rtw/rt_nonfinite.c
	$(CC) $(CFLAGS) -o "$@" "$<"


MW_ArduinoHWInit.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/src/MW_ArduinoHWInit.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


io_wrappers.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinobase/src/io_wrappers.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


arduinoAVRScheduler.o : /home/emil/Documents/MATLAB/SupportPackages/R2025b/toolbox/target/supportpackages/arduinotarget/scheduler/src/arduinoAVRScheduler.cpp
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(ALL_OBJS) : rtw_proj.tmw $(MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	echo "### PRODUCT = $(PRODUCT)"
	echo "### PRODUCT_TYPE = $(PRODUCT_TYPE)"
	echo "### BUILD_TYPE = $(BUILD_TYPE)"
	echo "### INCLUDES = $(INCLUDES)"
	echo "### DEFINES = $(DEFINES)"
	echo "### ALL_SRCS = $(ALL_SRCS)"
	echo "### ALL_OBJS = $(ALL_OBJS)"
	echo "### LIBS = $(LIBS)"
	echo "### MODELREF_LIBS = $(MODELREF_LIBS)"
	echo "### SYSTEM_LIBS = $(SYSTEM_LIBS)"
	echo "### TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	echo "### ASFLAGS = $(ASFLAGS)"
	echo "### CFLAGS = $(CFLAGS)"
	echo "### LDFLAGS = $(LDFLAGS)"
	echo "### SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	echo "### CPPFLAGS = $(CPPFLAGS)"
	echo "### CPP_LDFLAGS = $(CPP_LDFLAGS)"
	echo "### CPP_SHAREDLIB_LDFLAGS = $(CPP_SHAREDLIB_LDFLAGS)"
	echo "### ARFLAGS = $(ARFLAGS)"
	echo "### MEX_CFLAGS = $(MEX_CFLAGS)"
	echo "### MEX_CPPFLAGS = $(MEX_CPPFLAGS)"
	echo "### MEX_LDFLAGS = $(MEX_LDFLAGS)"
	echo "### MEX_CPPLDFLAGS = $(MEX_CPPLDFLAGS)"
	echo "### OBJCOPYFLAGS_BIN = $(OBJCOPYFLAGS_BIN)"
	echo "### OBJCOPYFLAGS_HEX = $(OBJCOPYFLAGS_HEX)"
	echo "### DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	echo "### EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	echo "### MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "### Deleting all derived files ..."
	$(RM) $(PRODUCT)
	$(RM) $(ALL_OBJS)
	$(RM) *.dep
	$(ECHO) "### Deleted all derived files."


