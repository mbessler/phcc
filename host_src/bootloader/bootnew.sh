#!/bin/sh

# load a new program into a An851 based bootloader with an851host:
# a 18F452 has 32k Flash, equivalent rows: 512
# bootloader takes up 0-1FF, => 512bytes or 8 rows of 64
#
./an851host --eraseflash --rows 504 --startaddr 0x0200 ; ./an851host --writeflash --hexfile $1 ; ./an851host --reset
