#BOOTROM_ADDR ?= 0x80000
BOOTROM_ADDR ?= 0x00100
OFFSET ?= 0x0

#TARGET ?= spi_uimage_loader
#TARGET ?= boot_loop
all: $(TARGET).vh $(TARGET).elf $(TARGET).hex

spi_uimage_loader.elf: spi_uimage_loader.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

boot_loop.elf: boot_loop.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

led_blink.elf: led_blink.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

mem_access.elf: mem_access.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

hexloader.elf: hexloader.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

hello.elf: hello.c
	or1k-elf-gcc -ggdb -mboard=nexys4ddr $< -o $@

%.vh: %.bin
	python ./wb_rom_gen.py $< > $@

%.bin: %.elf
	or1k-elf-objcopy -O binary $< $@

%.elf: %.S
	or1k-elf-gcc -DBOOTROM_ADDR=$(BOOTROM_ADDR) -nostdlib $< -o $@

%.ub: %.bin
	mkimage \
	-A or1k \
	-C none \
	-T standalone \
	-a $(OFFSET) \
	-e $(BOOTROM_ADDR) \
	-n '$@' \
	-d $< \
	$@

#%.hex: %.ub
#	or1k-elf-objcopy -I binary -O ihex $< $@

%.hex: %.elf
	or1k-elf-objcopy -O ihex $< $@

clean:
	rm spi_uimage_loader.elf $(TARGET).vh

