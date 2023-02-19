OPENDOG_TARGET = OpenDogV3
REMOTE_TARGET = RemoteDogV3
BUILD_DIR = build

OPENDOG_FILES = \
  $(BUILD_DIR)/$(OPENDOG_TARGET).elf \
  $(BUILD_DIR)/$(OPENDOG_TARGET).hex \
  $(BUILD_DIR)/$(OPENDOG_TARGET).bin

REMOTE_FILES = \
  $(BUILD_DIR)/$(REMOTE_TARGET).elf \
  $(BUILD_DIR)/$(REMOTE_TARGET).hex \
  $(BUILD_DIR)/$(REMOTE_TARGET).bin

all: $(OPENDOG_FILES) $(REMOTE_FILES)

clean:
	-rm -fR $(BUILD_DIR) ./tests/test_runner

ARDUINO_TEENSY_PACKAGE = $(HOME)/.arduino15/packages/teensy
ARDUINO_TEENSY_HOME = $(ARDUINO_TEENSY_PACKAGE)/hardware/avr/1.57.2
ARDUINO_TEENSY_TOOLS = $(ARDUINO_TEENSY_PACKAGE)/tools/teensy-tools/1.57.2
ARDUINO_TEENSY_LIBRARIES = $(ARDUINO_TEENSY_HOME)/libraries
ARDUINO_TEENSY_CORE = $(ARDUINO_TEENSY_HOME)/cores/teensy4
ARDUINO_LIBRARIES = $(HOME)/Arduino/libraries

PREFIX = arm-none-eabi-
ifdef ARM_GCC_PATH
ARM_CC = $(ARM_GCC_PATH)/$(PREFIX)gcc
ARM_CXX = $(ARM_GCC_PATH)/$(PREFIX)g++
ARM_AS = $(ARM_GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
ARM_CP = $(ARM_GCC_PATH)/$(PREFIX)objcopy
ARM_SZ = $(ARM_GCC_PATH)/$(PREFIX)size
else
ARM_CC = $(PREFIX)gcc
ARM_CXX = $(PREFIX)g++
ARM_AS = $(PREFIX)gcc -x assembler-with-cpp
ARM_CP = $(PREFIX)objcopy
ARM_SZ = $(PREFIX)size
endif
ARM_HEX = $(ARM_CP) -O ihex
ARM_BIN = $(ARM_CP) -O binary -S

ARM_AS_DEFS =

ARM_C_DEFS =  \
  -D__IMXRT1062__ \
  -DTEENSYDUINO=157 \
  -DARDUINO=10819 \
  -DARDUINO_TEENSY41 \
  -DF_CPU=600000000 \
  -DUSB_SERIAL \
  -DLAYOUT_US_ENGLISH

# AS includes
ARM_AS_INCLUDES = 

# C/C++ includes
ARM_C_INCLUDES =  \
  -I$(ARDUINO_TEENSY_CORE) \
  -I$(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3 \
  -I$(ARDUINO_TEENSY_LIBRARIES)/SPI \
  -I$(ARDUINO_TEENSY_LIBRARIES)/FlexCAN_T4 \
  -I$(ARDUINO_TEENSY_LIBRARIES)/ADC \
  -I$(ARDUINO_LIBRARIES)/RF24

# Teensy Arduino core source files
CORE_SOURCES = \
  $(ARDUINO_TEENSY_CORE)/memcpy-armv7m.S \
  $(ARDUINO_TEENSY_CORE)/memset.S \
  $(ARDUINO_TEENSY_CORE)/analog.c \
  $(ARDUINO_TEENSY_CORE)/bootdata.c \
  $(ARDUINO_TEENSY_CORE)/libc.c \
  $(ARDUINO_TEENSY_CORE)/debugprintf.c \
  $(ARDUINO_TEENSY_CORE)/clockspeed.c \
  $(ARDUINO_TEENSY_CORE)/interrupt.c \
  $(ARDUINO_TEENSY_CORE)/extmem.c \
  $(ARDUINO_TEENSY_CORE)/delay.c \
  $(ARDUINO_TEENSY_CORE)/digital.c \
  $(ARDUINO_TEENSY_CORE)/eeprom.c \
  $(ARDUINO_TEENSY_CORE)/keylayouts.c \
  $(ARDUINO_TEENSY_CORE)/fuse.c \
  $(ARDUINO_TEENSY_CORE)/nonstd.c \
  $(ARDUINO_TEENSY_CORE)/pwm.c \
  $(ARDUINO_TEENSY_CORE)/rtc.c \
  $(ARDUINO_TEENSY_CORE)/sm_alloc_valid.c \
  $(ARDUINO_TEENSY_CORE)/sm_calloc.c \
  $(ARDUINO_TEENSY_CORE)/sm_free.c \
  $(ARDUINO_TEENSY_CORE)/sm_hash.c \
  $(ARDUINO_TEENSY_CORE)/sm_malloc.c \
  $(ARDUINO_TEENSY_CORE)/sm_malloc_stats.c \
  $(ARDUINO_TEENSY_CORE)/sm_pool.c \
  $(ARDUINO_TEENSY_CORE)/sm_realloc.c \
  $(ARDUINO_TEENSY_CORE)/sm_realloc_i.c \
  $(ARDUINO_TEENSY_CORE)/sm_realloc_move.c \
  $(ARDUINO_TEENSY_CORE)/sm_szalloc.c \
  $(ARDUINO_TEENSY_CORE)/sm_util.c \
  $(ARDUINO_TEENSY_CORE)/sm_zalloc.c \
  $(ARDUINO_TEENSY_CORE)/startup.c \
  $(ARDUINO_TEENSY_CORE)/tempmon.c \
  $(ARDUINO_TEENSY_CORE)/usb.c \
  $(ARDUINO_TEENSY_CORE)/usb_desc.c \
  $(ARDUINO_TEENSY_CORE)/usb_joystick.c \
  $(ARDUINO_TEENSY_CORE)/usb_keyboard.c \
  $(ARDUINO_TEENSY_CORE)/usb_midi.c \
  $(ARDUINO_TEENSY_CORE)/usb_mouse.c \
  $(ARDUINO_TEENSY_CORE)/usb_mtp.c \
  $(ARDUINO_TEENSY_CORE)/usb_rawhid.c \
  $(ARDUINO_TEENSY_CORE)/usb_seremu.c \
  $(ARDUINO_TEENSY_CORE)/usb_serial.c \
  $(ARDUINO_TEENSY_CORE)/usb_serial2.c \
  $(ARDUINO_TEENSY_CORE)/usb_serial3.c \
  $(ARDUINO_TEENSY_CORE)/usb_touch.c \
  $(ARDUINO_TEENSY_CORE)/DMAChannel.cpp \
  $(ARDUINO_TEENSY_CORE)/EventResponder.cpp \
  $(ARDUINO_TEENSY_CORE)/AudioStream.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial6.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial3.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial7.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial4.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial1.cpp \
  $(ARDUINO_TEENSY_CORE)/CrashReport.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial5.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial2.cpp \
  $(ARDUINO_TEENSY_CORE)/HardwareSerial8.cpp \
  $(ARDUINO_TEENSY_CORE)/IPAddress.cpp \
  $(ARDUINO_TEENSY_CORE)/IntervalTimer.cpp \
  $(ARDUINO_TEENSY_CORE)/Print.cpp \
  $(ARDUINO_TEENSY_CORE)/Stream.cpp \
  $(ARDUINO_TEENSY_CORE)/Time.cpp \
  $(ARDUINO_TEENSY_CORE)/Tone.cpp \
  $(ARDUINO_TEENSY_CORE)/WMath.cpp \
  $(ARDUINO_TEENSY_CORE)/WString.cpp \
  $(ARDUINO_TEENSY_CORE)/main.cpp \
  $(ARDUINO_TEENSY_CORE)/new.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent1.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent2.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent3.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent4.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent5.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent6.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent7.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEvent8.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEventUSB1.cpp \
  $(ARDUINO_TEENSY_CORE)/serialEventUSB2.cpp \
  $(ARDUINO_TEENSY_CORE)/usb_audio.cpp \
  $(ARDUINO_TEENSY_CORE)/usb_flightsim.cpp \
  $(ARDUINO_TEENSY_CORE)/usb_inst.cpp \
  $(ARDUINO_TEENSY_CORE)/yield.cpp

# Teensy ST7735_t3 library source files
ST7735_t3_SOURCES = \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/glcdfont.c \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/st7735_t3_font_ComicSansMS.c \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/st7735_t3_font_Arial.c \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/ST7789_t3.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/ST7735_t3.cpp

# Teensy SPI library source files
SPI_SOURCES = \
  $(ARDUINO_TEENSY_LIBRARIES)/SPI/SPI.cpp

# Teensy ADC library source files
ADC_SOURCES = \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/AnalogBufferDMA.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/ADC_Module.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/ADC.cpp

# Arduino RF24 library source files
RF24_SOURCES = \
  $(ARDUINO_LIBRARIES)/RF24/RF24.cpp

OPENDOG_SOURCES = \
  ODriveCanTest.ino \
  CanInterfaceCommon.cpp \
  CanInterfaceTeensy4.cpp \
  JointDriver.cpp \
  Radio.cpp \
  SerialInteraction.cpp \
  VoltageMonitor.cpp \
  CanInterfaceSAME.cpp \
  Display.cpp \
  Kinematics.cpp \
  RobotDefinition.cpp \
  StepTrajectory.cpp \
  $(RF24_SOURCES) \
  $(ST7735_t3_SOURCES) \
  $(SPI_SOURCES) \
  $(ADC_SOURCES) \
  $(CORE_SOURCES)

REMOTE_SOURCES = \
  Remote.ino \
  RemoteInput.cpp \
  $(RF24_SOURCES) \
  $(SPI_SOURCES) \
  $(ADC_SOURCES) \
  $(CORE_SOURCES)

# Teensy Core linker script
LDSCRIPT = $(ARDUINO_TEENSY_CORE)/imxrt1062_t41.ld

# Flags for the ARM cross compiler
ARM_FPU = -mfpu=fpv5-d16
ARM_FLOAT-ABI = -mfloat-abi=hard
ARM_OPT = -O2
ARM_CPU = -mcpu=cortex-m7
ARM_MCU = $(ARM_CPU) -mthumb $(ARM_FPU) $(ARM_FLOAT-ABI)

ARM_ASFLAGS = \
  $(ARM_MCU) $(ARM_AS_DEFS) $(ARM_AS_INCLUDES) $(ARM_OPT) \
  -Wall -fdata-sections -ffunction-sections -g -MMD

ARM_CFLAGS  = \
  $(ARM_MCU) $(ARM_C_DEFS) $(ARM_C_INCLUDES) $(ARM_OPT) \
  -Wall -fdata-sections -ffunction-sections -g -nostdlib -MMD

ARM_CXXFLAGS = -std=gnu++14 -fno-exceptions -fno-rtti \
  -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing

# Generate dependency information
ARM_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

ARM_LIBS = -lc -lm
# library list used int he past for posterity if we need something from there
#-larm_cortexM7lfsp_math -lm -lstdc++ -lnosys

ARM_LIBDIR = 
ARM_LDFLAGS = \
  $(ARM_OPT) $(ARM_MCU) -specs=nano.specs -T$(LDSCRIPT) \
  $(ARM_LIBDIR) $(ARM_LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
  -Wl,--gc-sections,--relax

# Rules for cross compilation

# These rules create a list of all .o files we need to cross compile
# add .c files to objects
OPENDOG_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.c,%.o,$(filter %.c, $(OPENDOG_SOURCES)))))
# add .cpp files to objects
OPENDOG_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.cpp,%.o,$(filter %.cpp, $(OPENDOG_SOURCES)))))
# add .ino files to objects
OPENDOG_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.ino,%.o,$(filter %.ino, $(OPENDOG_SOURCES)))))
# add ASM files to objects
OPENDOG_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.S,%.o,$(filter %.S, $(OPENDOG_SOURCES)))))

# add .c files to objects
REMOTE_OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.c,%.o,$(filter %.c, $(REMOTE_SOURCES)))))
# add .cpp files to objects
REMOTE_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.cpp,%.o,$(filter %.cpp, $(REMOTE_SOURCES)))))
# add .ino files to objects
REMOTE_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.ino,%.o,$(filter %.ino, $(REMOTE_SOURCES)))))
# add ASM files to objects
REMOTE_OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(patsubst %.S,%.o,$(filter %.S, $(REMOTE_SOURCES)))))

vpath %.c $(sort $(dir $(filter %.c, $(OPENDOG_SOURCES) $(REMOTE_SOURCES))))
vpath %.cpp $(sort $(dir $(filter %.cpp, $(OPENDOG_SOURCES) $(REMOTE_SOURCES))))
vpath %.ino $(sort $(dir $(filter %.ino, $(OPENDOG_SOURCES) $(REMOTE_SOURCES))))
vpath %.S $(sort $(dir $(filter %.S, $(OPENDOG_SOURCES) $(REMOTE_SOURCES))))

# Generic rules how to cross compile each file extension
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(ARM_CC) -c $(ARM_CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR)
	$(ARM_CXX) -c $(ARM_CFLAGS) $(ARM_CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.ino Makefile | $(BUILD_DIR)
	$(ARM_CXX) -c -x c++ $(ARM_CFLAGS) $(ARM_CXXFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(ARM_AS) -c $(ARM_ASFLAGS) $< -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARM_HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARM_BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

# Rule how to link the $(OPENDOG_TARGET).elf
$(BUILD_DIR)/$(OPENDOG_TARGET).elf: $(OPENDOG_OBJECTS) Makefile
	$(ARM_CXX) $(OPENDOG_OBJECTS) $(ARM_LDFLAGS) -o $@
	$(ARM_SZ) $@
	$(ARDUINO_TEENSY_TOOLS)/teensy_size $@

# Rule how to flash the $(OPENDOG_TARGET).hex to the device
flash_opendog: $(BUILD_DIR)/$(OPENDOG_TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -v -s -w $<

# Rule how to link the $(REMOTE_TARGET).elf
$(BUILD_DIR)/$(REMOTE_TARGET).elf: $(REMOTE_OBJECTS) Makefile
	$(ARM_CXX) $(REMOTE_OBJECTS) $(ARM_LDFLAGS) -o $@
	$(ARM_SZ) $@
	$(ARDUINO_TEENSY_TOOLS)/teensy_size $@

# Rule how to flash the $(REMOTE_TARGET).hex to the device
flash_remote: $(BUILD_DIR)/$(REMOTE_TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -v -s -w $<

# Rules for unit tests, these run on the host platform
tests/test_runner: tests/*.cpp tests/*.h
	@g++ -g -Wall -Wextra -std=gnu++17 tests/*.cpp -o tests/test_runner

run_tests: tests/test_runner
	@./tests/test_runner

-include $(wildcard $(BUILD_DIR)/*.d)
