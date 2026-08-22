########################################################################
# STM32F103RE GNU Makefile converted from MDK-ARM/lgas.uvprojx
########################################################################

TARGET := lgas
BUILD_DIR := build

# Plain `make` defaults to parallel compilation. Override with JOBS=N or -jN.
JOBS ?= 8
ifeq ($(filter -j% --jobs%,$(MAKEFLAGS)),)
MAKEFLAGS += -j$(JOBS)
endif

GCC_PATH ?= C:/Users/LGZN2/STM32Cube/Repository/Packs/STMicroelectronics/X-CUBE-AI/10.2.0/Utilities/windows/gnu-tools-for-stm32/tools/bin/
PREFIX := $(GCC_PATH)arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc -x assembler-with-cpp
CP := $(PREFIX)objcopy
SZ := $(PREFIX)size
NM := $(PREFIX)nm
OBJDUMP := $(PREFIX)objdump

# Keep the normal Windows toolchain default while allowing an extracted Linux
# toolchain to provide an absolute specs file when its layout requires it.
NANO_SPECS ?= nano.specs
ARM_INCLUDE ?=

OPENOCD ?= E:/OpenOCD-20260302-0.12.0/bin/openocd.exe
OPENOCD_SCRIPTS ?= E:/OpenOCD-20260302-0.12.0/share/openocd/scripts
OPENOCD_INTERFACE ?= interface/stlink.cfg
OPENOCD_TARGET ?= target/stm32f1x.cfg

CPU := -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
C_DEFS := -DUSE_HAL_DRIVER -DSTM32F103xE
C_INCLUDES := \
  $(if $(strip $(ARM_INCLUDE)),-isystem $(ARM_INCLUDE)) \
  -ICore/Inc \
  -IDEVICE/include \
  -IUSB_DEVICE/App \
  -IUSB_DEVICE/Target \
  -IDrivers/STM32F1xx_HAL_Driver/Inc \
  -IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
  -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc \
  -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc \
  -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
  -IDrivers/CMSIS/Include

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/stm32f1xx_it.c \
  Core/Src/stm32f1xx_hal_msp.c \
  Core/Src/system_stm32f1xx.c \
  Core/Src/uart.c \
  Core/Src/syscalls.c \
  DEVICE/task/loop_profiler.c \
  USB_DEVICE/App/usb_device.c \
  USB_DEVICE/App/usbd_desc.c \
  USB_DEVICE/App/usbd_cdc_if.c \
  USB_DEVICE/Target/usbd_conf.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pcd_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usb.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dac.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dac_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rtc_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c \
  Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c \
  DEVICE/rs485/rs485.c \
  DEVICE/task/dgusii.c \
  DEVICE/rs485/rs485_h2.c \
  DEVICE/rtc/rtc.c \
  DEVICE/fan/fan.c \
  DEVICE/pump/pump.c \
  DEVICE/modbus/modbus_slave.c \
  DEVICE/modbus/modbus_master.c \
  DEVICE/modbus/modbus_master_h2.c \
  DEVICE/math/stats.c \
  DEVICE/math/fit.c \
  DEVICE/math/hmi_text_gbk.c \
  DEVICE/math/flash_storage.c \
  DEVICE/math/dev_addr.c \
  DEVICE/task/usb_task.c \
  DEVICE/task/app_scheduler.c \
  DEVICE/task/app_tasks.c \
  DEVICE/task/dataup.c \
  DEVICE/task/bus485_task.c \
  DEVICE/task/start_task.c \
  DEVICE/task/fault_monitor.c \
  DEVICE/task/HMI_task.c \
  DEVICE/sensor/CH4.c \
  DEVICE/sensor/C2H6.c \
  DEVICE/dg408/dg408.c \
  DEVICE/sensor/C2H2.c \
  DEVICE/sensor/H2.c \
  DEVICE/DAC/DAC.c \
  DEVICE/solenoid/solenoid.c

ASM_SOURCES := Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xe.s

CFLAGS := $(CPU) $(C_DEFS) $(C_INCLUDES) \
  -std=gnu11 -O2 -g3 -Wall -ffunction-sections -fdata-sections \
  -fno-strict-aliasing -MMD -MP
CFLAGS += $(EXTRA_CFLAGS)
ASFLAGS := $(CPU) $(C_DEFS) $(C_INCLUDES) -g3

LDSCRIPT := STM32F103RE_FLASH.ld
LDFLAGS := $(CPU) --specs=$(NANO_SPECS) -T$(LDSCRIPT) \
  -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -Wl,--no-warn-rwx-segments
LIBS := -Wl,--start-group -lc -lm -Wl,--end-group -u _printf_float

C_OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
ASM_OBJECTS := $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
OBJECTS := $(C_OBJECTS) $(ASM_OBJECTS)
DEPS := $(C_OBJECTS:.o=.d)

# Windows CMD helpers.
ifeq ($(OS),Windows_NT)
define make_dir
	@if not exist "$(subst /,\\,$(dir $@))" mkdir "$(subst /,\\,$(dir $@))"
endef
RM_BUILD = if exist "$(subst /,\\,$(BUILD_DIR))" rmdir /S /Q "$(subst /,\\,$(BUILD_DIR))"
else
define make_dir
	@mkdir -p "$(dir $@)"
endef
RM_BUILD = rm -rf "$(BUILD_DIR)"
endif

.PHONY: all clean size flash verify
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile
	$(make_dir)
	$(CC) -c $(CFLAGS) -MF"$(@:.o=.d)" $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	$(make_dir)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(LDSCRIPT)
	$(make_dir)
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@
	$(SZ) $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(CP) -O binary -S $< $@

clean:
	@$(RM_BUILD)

size: $(BUILD_DIR)/$(TARGET).elf
	$(SZ) $<

verify: all
	$(OBJDUMP) -h $(BUILD_DIR)/$(TARGET).elf
	$(NM) -n $(BUILD_DIR)/$(TARGET).elf > $(BUILD_DIR)/$(TARGET).symbols.txt

flash: $(BUILD_DIR)/$(TARGET).elf
	$(OPENOCD) -s "$(OPENOCD_SCRIPTS)" -f "$(OPENOCD_INTERFACE)" -f "$(OPENOCD_TARGET)" -c "program $(abspath $<) verify reset exit"

-include $(DEPS)
