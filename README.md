# Bootloader build for NXP i.MX 93 11x11 EVK

Makefile for building flash.bin.

Based on NXP Linux Release
[LF6.12.34_2.1.0 (11 nov 2025)](https://www.nxp.com/docs/en/release-note/RN00210.pdf).

[imx93 eval kit doc](uboot-imx/doc/board/nxp/imx93_11x11_evk.rst)

Toolchain can be downloaded from [ARM](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads):

```sh
wget https://developer.arm.com/-/media/Files/downloads/gnu/14.3.rel1/binrel/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-linux-gnu.tar.xz
tar xf arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-linux-gnu.tar.xz
```

## Dependencies

The following are automatically downloaded by the Makefile. Note: The firmware EULAs are auto-accepted.

| Dependency | Release version | Description |
|------------|----------------|-------------|
| DDR firmware (lpddr4*.bin) | firmware-imx-8.29-8741a3b.bin | i.MX Firmware including firmware for VPU, DDR, EPDC, HDMI, DP (Display Port), and SDMA. Only the Synopsys DDR firmware is used here. |
| AHAB container - ELE Firmware (mx93a1-ahab-container.img) | firmware-ele-imx-2.0.3-286c884.bin | NXP ELE firmware for the EdgeLock Secure Enclave (ELE), a dedicated security processor on NXP microcontrollers. ELE handles overall boot-time security, including decryption and integrity checks, and AHAB specifically manages the authentication of images to prevent tampering. |
| ATF - Atmel Trusted Firmware (bl31.bin) | lf-6.12.34-2.1.0 | NXP i.MX Trusted Firmware-A (ATF) is a secure boot and runtime firmware for NXP's i.MX processors that implements Arm's Trusted Firmware-A (TF-A) specification. After completing its tasks, ATF hands over control to the next stage of the bootloader, such as U-Boot, which then boots the Linux kernel or other operating systems. |
| imx-mkimage | lf-6.12.34-2.1.0 | This tool takes individual components like the ARM Trusted Firmware, Secure World components (SCFW), DDR firmware, and a bootloader (like U-Boot or an SPL) and combines them into a single, executable binary file that the i.MX processor can boot from. |

## Build flash.bin

```sh
export CROSS_COMPILE=$PWD/arm-gnu-toolchain-14.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-
export ARTIFACTS_DIR=$HOME/.artifact-downloads && mkdir -p $ARTIFACTS_DIR
sudo apt install efitools
make -j $(nproc)
```
