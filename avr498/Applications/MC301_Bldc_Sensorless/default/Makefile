###############################################################################
# Makefile for the project bldc_sensorless_dbg
###############################################################################

## General Flags
PROJECT = bldc_sensorless_dbg
MCU = attiny861
TARGET = bldc_sensorless_dbg.elf
CC = avr-gcc.exe

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99 -O0 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=bldc_sensorless_dbg.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\..\..\common\lib_mcu\twi" -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\..\..\common\USI_TWI_slave" -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\conf" -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\..\..\common" -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\..\..\common\lib_mcu" -I"D:\SVN\svnssg\products\avr8\tiny861\software\trunk\Deliveries\AVR498\avr498-1_0_1\avr498-1_0_1\Applications\MC301_Bldc_Sensorless\." 

## Objects that must be built in order to link
OBJECTS = main.o mc_control.o mc_drv.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) bldc_sensorless_dbg.hex bldc_sensorless_dbg.eep bldc_sensorless_dbg.lss size

## Compile
main.o: ../main.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

mc_control.o: ../mc_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

mc_drv.o: ../mc_drv.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) bldc_sensorless_dbg.elf dep/* bldc_sensorless_dbg.hex bldc_sensorless_dbg.eep bldc_sensorless_dbg.lss bldc_sensorless_dbg.map


## Other dependencies
-include $(shell mkdir dep 2>/dev/null) $(wildcard dep/*)

