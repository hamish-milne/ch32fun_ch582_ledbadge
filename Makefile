all : flash

TARGET:=ledbadge2
TARGET_MCU:=CH582
TARGET_MCU_PACKAGE:=CH582F
PREFIX:=riscv64-unknown-none-elf

ADDITIONAL_C_FILES:=./badapple.c

include ./ch32fun/ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
