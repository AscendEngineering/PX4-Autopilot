# RP2350 bare-metal probe (phase 1). Standalone; not part of the PX4 CMake build.
#
#   make            build/probe_bl.{elf,bin,uf2} and build/probe_app.{elf,bin,uf2}
#   make check      layout checks on both .bin files and a UF2 round trip
#   make clean
#
# PROBE_CPU_HZ is the assumed processor clock for SysTick (no PLL is set up).
# Calibrate on the bench from systick_irqs against a wall clock.

CROSS   ?= arm-none-eabi-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size
PYTHON  ?= python3

PROBE_CPU_HZ ?= 12000000
# BOARD=uavfc (default): LEDs on GPIO0/1 active low. BOARD=pico2: GPIO25 active high.
BOARD ?= uavfc
ifeq ($(BOARD),pico2)
BOARD_FLAGS := -DLED_BLUE_GPIO=25u -DLED_GREEN_GPIO=16u -DLED_ACTIVE_HIGH=1
else
BOARD_FLAGS :=
endif
UF2CONV := ../../../../Tools/uf2conv.py

BUILD := build
SRCS  := start.c rom.c flash.c probe.c

CFLAGS := -mcpu=cortex-m33 -mthumb -mfloat-abi=soft \
          -O2 -g -std=c11 -ffreestanding -fno-builtin -nostdlib \
          -fno-tree-loop-distribute-patterns \
          -ffunction-sections -fdata-sections \
          -Wall -Wextra -Werror -Wundef \
          -DPROBE_CPU_HZ=$(PROBE_CPU_HZ)u $(BOARD_FLAGS)

LDFLAGS := -mcpu=cortex-m33 -mthumb -mfloat-abi=soft -nostartfiles -nostdlib \
           -Wl,--gc-sections

# role name, flash origin, flash length, PROBE_ROLE_APP
ROLES := bl app
bl_ORIGIN  := 0x10000000
bl_LENGTH  := 128K
bl_ROLE    := 0
app_ORIGIN := 0x10020000
app_LENGTH := 3904K
app_ROLE   := 1

all: $(foreach r,$(ROLES),$(BUILD)/probe_$(r).elf $(BUILD)/probe_$(r).bin $(BUILD)/probe_$(r).uf2)
	@$(SIZE) $(BUILD)/probe_bl.elf $(BUILD)/probe_app.elf

define ROLE_RULES
$(BUILD)/$(1)/%.o: %.c probe.h rom.h flash.h Makefile $(BUILD)/config | $(BUILD)/$(1)
	$$(CC) $$(CFLAGS) -DPROBE_ROLE_APP=$($(1)_ROLE) -c $$< -o $$@

$(BUILD)/probe_$(1).ld: probe.ld Makefile $(BUILD)/config | $(BUILD)
	$$(CC) -E -P -x c -DFLASH_ORIGIN=$($(1)_ORIGIN) -DFLASH_LENGTH=$($(1)_LENGTH) $$< -o $$@

$(BUILD)/probe_$(1).elf: $(addprefix $(BUILD)/$(1)/,$(SRCS:.c=.o)) $(BUILD)/probe_$(1).ld
	$$(CC) $$(LDFLAGS) -T $(BUILD)/probe_$(1).ld -Wl,-Map,$(BUILD)/probe_$(1).map \
	    $(addprefix $(BUILD)/$(1)/,$(SRCS:.c=.o)) -lgcc -o $$@

$(BUILD)/probe_$(1).uf2: $(BUILD)/probe_$(1).bin $(UF2CONV) Makefile
	$$(PYTHON) $(UF2CONV) --base $($(1)_ORIGIN) $$< -o $$@
endef
$(foreach r,$(ROLES),$(eval $(call ROLE_RULES,$(r))))

$(BUILD)/%.bin: $(BUILD)/%.elf
	$(OBJCOPY) -O binary $< $@

$(BUILD) $(BUILD)/bl $(BUILD)/app:
	mkdir -p $@

# Preserve the timestamp on unchanged settings; switching back to an earlier
# frequency must also rebuild the shared output paths.
$(BUILD)/config: FORCE | $(BUILD)
	@printf '%s\n' 'CC=$(CC)' 'CFLAGS=$(CFLAGS)' 'LDFLAGS=$(LDFLAGS)' > $@.tmp
	@cmp -s $@.tmp $@ || cp $@.tmp $@

FORCE:

check: all
	$(PYTHON) $(UF2CONV) --self-test
	$(PYTHON) check_image.py --role bl  $(BUILD)/probe_bl.bin
	$(PYTHON) check_image.py --role app $(BUILD)/probe_app.bin
	$(PYTHON) $(UF2CONV) --decode $(BUILD)/probe_bl.uf2 -o $(BUILD)/probe_bl.rt.bin
	cmp -n $$(stat -c %s $(BUILD)/probe_bl.bin) $(BUILD)/probe_bl.bin $(BUILD)/probe_bl.rt.bin

clean:
	rm -rf $(BUILD)

.PHONY: all check clean FORCE
