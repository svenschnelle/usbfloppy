CROSS_COMPILE?=arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy

ARMCPU=cortex-m3
STM32MCU=STM32F103x6
LINKERSCRIPT=FLASH.ld

OBJS=	device/generic.o\
	device/stm32_it.o\
	device/terminal.o\
	lib/USB-FS-Device_Lib/usb_core.o\
	lib/USB-FS-Device_Lib/usb_init.o\
	lib/USB-FS-Device_Lib/usb_int.o\
	lib/USB-FS-Device_Lib/usb_mem.o\
	lib/USB-FS-Device_Lib/usb_regs.o\
	lib/USB-FS-Device_Lib/usb_sil.o\
	lib/USB_Composite_MSC-CDC/hw_config.o\
	lib/USB_Composite_MSC-CDC/mass_mal.o\
	lib/USB_Composite_MSC-CDC/memory.o\
	lib/USB_Composite_MSC-CDC/scsi_data.o\
	lib/USB_Composite_MSC-CDC/usb_bot.o\
	lib/USB_Composite_MSC-CDC/usb_desc.o\
	lib/USB_Composite_MSC-CDC/usb_endp.o\
	lib/USB_Composite_MSC-CDC/usb_istr.o\
	lib/USB_Composite_MSC-CDC/usb_prop.o\
	lib/USB_Composite_MSC-CDC/usb_pwr.o\
	lib/USB_Composite_MSC-CDC/usb_scsi.o\
	lib/USB_Composite_MSC-CDC/virtualComPort.o\
	lib/cmsis/system_stm32f10x.o\
	lib/spl/src/misc.o\
	lib/spl/src/stm32f10x_adc.o\
	lib/spl/src/stm32f10x_bkp.o\
	lib/spl/src/stm32f10x_can.o\
	lib/spl/src/stm32f10x_cec.o\
	lib/spl/src/stm32f10x_crc.o\
	lib/spl/src/stm32f10x_dac.o\
	lib/spl/src/stm32f10x_dbgmcu.o\
	lib/spl/src/stm32f10x_dma.o\
	lib/spl/src/stm32f10x_exti.o\
	lib/spl/src/stm32f10x_flash.o\
	lib/spl/src/stm32f10x_fsmc.o\
	lib/spl/src/stm32f10x_gpio.o\
	lib/spl/src/stm32f10x_i2c.o\
	lib/spl/src/stm32f10x_iwdg.o\
	lib/spl/src/stm32f10x_pwr.o\
	lib/spl/src/stm32f10x_rcc.o\
	lib/spl/src/stm32f10x_rtc.o\
	lib/spl/src/stm32f10x_sdio.o\
	lib/spl/src/stm32f10x_spi.o\
	lib/spl/src/stm32f10x_tim.o\
	lib/spl/src/stm32f10x_usart.o\
	lib/spl/src/stm32f10x_wwdg.o\
	lib/terminal-server/term-srv.o\
	Startup/startup_stm32f103c8tx.o\
	main.o

CFLAGS=-std=c99 -Wall -fno-common -mthumb -mcpu=$(ARMCPU) -D$(STM32MCU) -DSTM32F10X_LD\
	-Ilib/cmsis\
	-Ilib/spl/inc\
	-Idevice\
	-Ilib/USB_Composite_MSC-CDC\
	-Ilib/USB-FS-Device_Lib\
	-Ilib/terminal-server

LDFLAGS=-T$(LINKERSCRIPT) -mcpu=$(ARMCPU) -lc

ifeq ($(V),1)
QUIET=
QECHO=@\#
else
QUIET=@
QECHO=@echo
endif

all: usbfloppy.bin

usbfloppy.bin: usbfloppy.elf

%.bin:	%.elf
	$(QECHO) "OBJCOPY $@"
	$(QUIET)$(OBJCOPY) -R .stack -O binary $< $@

%.elf: $(OBJS)
	$(QECHO) "LD $@"
	$(QUIET)$(CC) $(LDFLAGS) -o $@ $^

%.o:	%.c
	$(QECHO) "CC $@"
	$(QUIET)$(CC) $(CFLAGS) -c -o $@ $<

%.o:	%.s
	$(QECHO) "CC $@"
	$(QUIET)$(CC) $(CFLAGS) -c -o $@ $<

clean:
	$(QECHO) "CLEAN"
	$(QUIET)rm -f usbfloppy.bin usbfloppy.elf $(OBJS)
