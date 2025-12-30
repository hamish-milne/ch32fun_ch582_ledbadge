all : flash

TARGET:=ledbadge2
TARGET_MCU:=CH582
TARGET_MCU_PACKAGE:=CH582F
PREFIX:=riscv64-unknown-none-elf

include ../ch32fun/ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
