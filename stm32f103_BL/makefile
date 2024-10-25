
NAME=stm32f103_BL

# GCC_DIR=/home/fademike/Programs/STMicroelectronics/stm32cubeide_1.7.0/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.9-2020-q2-update.linux64_2.0.0.202105311346/tools/bin
GCC_DIR=/usr/bin
GCC = $(GCC_DIR)/arm-none-eabi-gcc
OBJCOPY = $(GCC_DIR)/arm-none-eabi-objcopy
SIZE = $(GCC_DIR)/arm-none-eabi-size
OUTDIR = out

LD_SCRIPT = STM32F103C8Tx_FLASH.ld

SRCDIR = Src
SRCDIR += Drivers/STM32F1xx_HAL_Driver/Src
SRCDIR += Src/modem
SRCDIR += startup

INC = -I ./Inc 
INC += -I ./Drivers/CMSIS/Include/ 
INC += -I ./Drivers/CMSIS/Device/ST/STM32F1xx/Include/ 
INC += -I ./Src/ 
INC += -I ./Src/modem/ 
INC += -I ./Drivers/STM32F1xx_HAL_Driver/Inc/ 
INC += -I ./Drivers/STM32F1xx_HAL_Driver/Inc/Legacy/
INC += -I ./Inc/mavlink/common

DEFS=-DSTM32F103xB
# OPTIMIZATION=-O0 -g3 -DDEBUG
OPTIMIZATION=-Os
# OPTIMIZATION += -g3 -DDEBUG

CCFLAGS =-mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER $(DEFS) $(OPTIMIZATION) -Wall -fmessage-length=0 -ffunction-sections -c 

LDFLAGS=-mcpu=cortex-m3 -mthumb -mfloat-abi=soft -specs=nosys.specs -specs=nano.specs -T$(LD_SCRIPT) -Wl,--gc-sections -lm $(OPTIMIZATION)
#LDFLAGS+=-Wl,--print-memory-usage
#LDFLAGS+=Wl,-Map=$(OUTDIR)/output.map

FILES_C := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.c)) 
FILES_S := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.s)) 

OBJECTS = $(patsubst %.c, $(OUTDIR)/%.o,${FILES_C})
OBJECTS += $(patsubst %.s, $(OUTDIR)/%.o,${FILES_S})


all: makefile $(OUTDIR)/$(NAME).bin

clean:
	-rm -Rf $(OUTDIR)/
load: $(OUTDIR)/$(NAME).elf
	@echo load
	~/Programs/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=swd -w $(OUTDIR)/$(NAME).bin 0x08000000 --verify -rst

debug: $(OUTDIR)/$(NAME).elf
	@echo load
	~/Programs/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI -c port=swd -w $(OUTDIR)/$(NAME).bin 0x08000000 --verify -rst

$(OUTDIR)/%.o: %.c
	@-mkdir -p $(dir $@)
	@$(GCC) $(CCFLAGS) $(INC) -c $< -o $@
$(OUTDIR)/%.o: %.s
	@-mkdir -p  $(dir $@)
	@$(GCC) $(CCFLAGS) $(INC) -c $< -o $@
	
$(OUTDIR)/$(NAME).elf: $(OBJECTS)
	@$(GCC) $(INC) $(LDFLAGS) $(OBJECTS) -o $@
	@echo link $(NAME) OK

$(OUTDIR)/$(NAME).bin: $(OUTDIR)/$(NAME).elf
	@$(OBJCOPY) -O binary $< $@
	@echo compiled $(NAME) OK
	@$(SIZE) $(OUTDIR)/$(NAME).elf
	
	
.PHONY: all
