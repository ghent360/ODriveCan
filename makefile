TARGET = OpenDogV3
BUILD_DIR = build

ARDUINO_TEENSY_PACKAGE = $(HOME)/.arduino15/packages/teensy
ARDUINO_TEENSY_HOME = $(ARDUINO_TEENSY_PACKAGE)/hardware/avr/1.57.1
ARDUINO_TEENSY_TOOLS = $(ARDUINO_TEENSY_PACKAGE)/tools/teensy-tools/1.57.1
ARDUINO_TEENSY_LIBRARIES = $(ARDUINO_TEENSY_HOME)/libraries
ARDUINO_TEENSY_CORE = $(ARDUINO_TEENSY_HOME)/cores/teensy4
ARDUINO_LIBRARIES = $(HOME)/Arduino/libraries

PREFIX = arm-none-eabi-
ifdef GCC_PATH
ARM_CC = $(GCC_PATH)/$(PREFIX)gcc
ARM_CXX = $(GCC_PATH)/$(PREFIX)g++
ARM_AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
ARM_CP = $(GCC_PATH)/$(PREFIX)objcopy
ARM_SZ = $(GCC_PATH)/$(PREFIX)size
else
ARM_CC = $(PREFIX)gcc
ARM_CXX = $(PREFIX)g++
ARM_AS = $(PREFIX)gcc -x assembler-with-cpp
ARM_CP = $(PREFIX)objcopy
ARM_SZ = $(PREFIX)size
endif
ARM_HEX = $(ARM_CP) -O ihex
ARM_BIN = $(ARM_CP) -O binary -S

ARM_FPU = -mfpu=fpv5-d16
ARM_FLOAT = -mfloat-abi=hard 
ARM_OPT = -O2
ARM_CPU = -mcpu=cortex-m7
ARM_MCU = $(ARM_CPU) -mthumb $(ARM_FPU) $(ARM_FLOAT-ABI)

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

# C includes
ARM_C_INCLUDES =  \
  -I$(ARDUINO_TEENSY_CORE) \
  -I$(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3 \
  -I$(ARDUINO_TEENSY_LIBRARIES)/SPI \
  -I$(ARDUINO_TEENSY_LIBRARIES)/FlexCAN_T4 \
  -I$(ARDUINO_TEENSY_LIBRARIES)/ADC \
  -I$(ARDUINO_LIBRARIES)/RF24

ASM_SOURCES = \
  $(ARDUINO_TEENSY_CORE)/memcpy-armv7m.S \
  $(ARDUINO_TEENSY_CORE)/memset.S

C_SOURCES = \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/glcdfont.c \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/st7735_t3_font_ComicSansMS.c \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/st7735_t3_font_Arial.c \
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
  $(ARDUINO_TEENSY_CORE)/usb_touch.c

CXX_SOURCES = \
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
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/ST7789_t3.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ST7735_t3/ST7735_t3.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/SPI/SPI.cpp \
  $(ARDUINO_LIBRARIES)/RF24/RF24.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/AnalogBufferDMA.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/ADC_Module.cpp \
  $(ARDUINO_TEENSY_LIBRARIES)/ADC/ADC.cpp \
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

SKETCH = \
  ODriveCanTest.ino

LDSCRIPT = $(ARDUINO_TEENSY_CORE)/imxrt1062_t41.ld

ARM_ASFLAGS = $(ARM_MCU) $(ARM_AS_DEFS) $(ARM_AS_INCLUDES) $(ARM_OPT) -Wall -fdata-sections -ffunction-sections

ARM_CFLAGS += $(ARM_MCU) $(ARM_C_DEFS) $(ARM_C_INCLUDES) $(ARM_OPT) -Wall -fdata-sections -ffunction-sections

ARM_CFLAGS += -g -nostdlib -MMD
ARM_CXXFLAGS = -std=gnu++14 -fno-exceptions -fpermissive -fno-rtti \
  -fno-threadsafe-statics -felide-constructors -Wno-error=narrowing

# Generate dependency information
ARM_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

ARM_LIBS = -lc -lm -lnosys
#-larm_cortexM7lfsp_math -lm -lstdc++
ARM_LIBDIR = 
ARM_LDFLAGS = $(ARM_OPT) $(ARM_MCU) -specs=nano.specs -T$(LDSCRIPT) $(ARM_LIBDIR) $(ARM_LIBS) \
  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections,--relax

# add .c files to objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# add .cpp files to objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CXX_SOURCES:.cpp=.o)))
vpath %.cpp $(sort $(dir $(CXX_SOURCES)))
# add .ino files to objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(SKETCH:.ino=.o)))
vpath %.ino $(sort $(dir $(SKETCH)))
# add ASM files to objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c makefile | $(BUILD_DIR)
	$(ARM_CC) -c $(ARM_CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.cpp makefile | $(BUILD_DIR)
	$(ARM_CXX) -c $(ARM_CFLAGS) $(ARM_CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.ino makefile | $(BUILD_DIR)
	$(ARM_CXX) -c -x c++ $(ARM_CFLAGS) $(ARM_CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.S makefile | $(BUILD_DIR)
	$(ARM_AS) -c $(ARM_CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) makefile
	$(ARM_CXX) $(OBJECTS) $(ARM_LDFLAGS) -o $@
	$(ARM_SZ) $@
	$(ARDUINO_TEENSY_TOOLS)/teensy_size $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARM_HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(ARM_BIN) $< $@	
	
$(BUILD_DIR):
	mkdir $@		

clean:
	-rm -fR $(BUILD_DIR)
  
tests/test_runner: tests/*.cpp tests/*.h
	@g++ -g -Wall -Wextra -std=gnu++17 tests/*.cpp -o tests/test_runner

run_tests: tests/test_runner
	@./tests/test_runner

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

flash: $(BUILD_DIR)/$(TARGET).hex
	teensy_loader_cli --mcu=TEENSY41 -v -s -w $<

-include $(wildcard $(BUILD_DIR)/*.d)
