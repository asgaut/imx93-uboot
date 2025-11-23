# Skip checks if doing 'make clean'
ifneq ($(MAKECMDGOALS),clean)
ifndef CROSS_COMPILE
$(error CROSS_COMPILE is not defined)
endif
ifndef ARTIFACTS_DIR
$(error ARTIFACTS_DIR is not defined)
endif
endif

# Check if ARTIFACTS_DIR exists
ifeq ($(wildcard $(ARTIFACTS_DIR)),)
$(error ARTIFACTS_DIR directory '$(ARTIFACTS_DIR)' does not exist. Please create it before running make.)
endif

# Variables
FIRMWARE_IMX_TAG := firmware-imx-8.29-8741a3b
FIRMWARE_AHAB_TAG := firmware-ele-imx-2.0.3-286c884
NXP_BSP_RELEASE := lf-6.12.34-2.1.0

FIRMWARE_IMX_BIN := $(ARTIFACTS_DIR)/$(FIRMWARE_IMX_TAG).bin
FIRMWARE_AHAB_BIN := $(ARTIFACTS_DIR)/$(FIRMWARE_AHAB_TAG).bin
ATF_TAR := $(ARTIFACTS_DIR)/imx-atf-$(NXP_BSP_RELEASE).tar.gz
MKIMAGE_TAR := $(ARTIFACTS_DIR)/imx-mkimage-$(NXP_BSP_RELEASE).tar.gz

# Output directories
UBOOT_OUT := output-uboot
ATF_OUT := output-atf
MKIMAGE_OUT := output-mkimage

default: $(MKIMAGE_OUT)/iMX93/flash.bin

downloads: $(FIRMWARE_IMX_BIN) $(FIRMWARE_AHAB_BIN) $(ATF_TAR) $(MKIMAGE_TAR)

## DDR firmware
# touch the target to ensure the target destination file timestamp is later that the .bin file
$(FIRMWARE_IMX_TAG)/firmware/ddr: $(FIRMWARE_IMX_BIN)
	chmod +x $<
	$< --force --auto-accept
	touch $@

$(FIRMWARE_IMX_BIN):
	wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/$(FIRMWARE_IMX_TAG).bin -O $@

## AHAB container
$(FIRMWARE_AHAB_TAG)/mx93a1-ahab-container.img: $(FIRMWARE_AHAB_BIN)
	chmod +x $<
	$< --force --auto-accept
	touch $@

$(FIRMWARE_AHAB_BIN):
	wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/$(FIRMWARE_AHAB_TAG).bin -O $@

## ATF build
$(ATF_OUT)/imx93/release/bl31.bin: $(ATF_TAR)
	mkdir -p imx-atf
	tar xf $< -C imx-atf --strip-components=1
	$(MAKE) -C imx-atf PLAT=imx93 bl31 BUILD_BASE=../$(ATF_OUT)

$(ATF_TAR):
	wget https://github.com/nxp-imx/imx-atf/archive/refs/tags/$(NXP_BSP_RELEASE).tar.gz -O $@

## imx-mkimage
$(MKIMAGE_OUT): $(MKIMAGE_TAR)
	mkdir -p $(MKIMAGE_OUT)
	tar xf $< -C $(MKIMAGE_OUT) --strip-components=1
	echo '#define MKIMAGE_COMMIT 0xdeadbeef' > $(MKIMAGE_OUT)/src/build_info.h

$(MKIMAGE_TAR):
	wget https://github.com/nxp-imx/imx-mkimage/archive/refs/tags/$(NXP_BSP_RELEASE).tar.gz -O $@

## U-Boot config
$(UBOOT_OUT)/.config:
	$(MAKE) -C uboot-imx O=../$(UBOOT_OUT) imx93_11x11_evk_defconfig

## U-Boot build
$(UBOOT_OUT)/u-boot.bin $(UBOOT_OUT)/spl/u-boot-spl.bin: $(UBOOT_OUT)/.config
	$(MAKE) -C $(UBOOT_OUT) clean
	$(MAKE) -C $(UBOOT_OUT)

## Create bootloader (flash.bin)
$(MKIMAGE_OUT)/iMX93/flash.bin: $(MKIMAGE_OUT) $(UBOOT_OUT)/u-boot.bin $(UBOOT_OUT)/spl/u-boot-spl.bin \
	$(FIRMWARE_AHAB_TAG)/mx93a1-ahab-container.img \
	$(FIRMWARE_IMX_TAG)/firmware/ddr \
	$(ATF_OUT)/imx93/release/bl31.bin

	cp $(FIRMWARE_AHAB_TAG)/mx93a1-ahab-container.img $(MKIMAGE_OUT)/iMX93
	cp $(FIRMWARE_IMX_TAG)/firmware/ddr/synopsys/lpddr4*.bin $(MKIMAGE_OUT)/iMX93
	cp $(ATF_OUT)/imx93/release/bl31.bin $(MKIMAGE_OUT)/iMX93
	cp $(UBOOT_OUT)/spl/u-boot-spl.bin $(MKIMAGE_OUT)/iMX93
	cp $(UBOOT_OUT)/u-boot.bin $(MKIMAGE_OUT)/iMX93
	@echo "Creating flash.bin for iMX93..."
	(cd $(MKIMAGE_OUT) && $(MAKE) SOC=iMX93 flash_singleboot)

# flash.bin works with nxp uuu (set imx93 evk to boot from USB first):
# Run: .\uuu.exe \\wsl.localhost\Ubuntu-24.04\home\username\bootloader\output-mkimage\iMX93\flash.bin
# then reset the board.

## Clean
.PHONY: clean
clean:
	rm -rf $(FIRMWARE_IMX_TAG) $(FIRMWARE_AHAB_TAG) imx-atf $(MKIMAGE_OUT) $(UBOOT_OUT) $(ATF_OUT)
