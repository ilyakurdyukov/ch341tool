## CH341A/B progammer command line tool

The [ch341prog](https://github.com/setarcos/ch341prog) doesn't suit my needs, as it don't support precise page erase, only the entire chip. So I wrote my own tool.

Only `SPI25xx` mode is supported.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, USE AT YOUR OWN RISK!

### Build

There are two options:

1. Using `libusb` for Linux and Windows (MSYS2):  
Use `make`, `libusb/libusb-dev` packages must be installed.

* For Windows users - please read how to install a [driver](https://github.com/libusb/libusb/wiki/Windows#driver-installation) for `libusb`.

2. Using the USB serial:  
Use `make LIBUSB=0`.
If you're using this mode, you must initialize the USB serial driver before using the tool (every boot):
```
$ sudo modprobe ftdi_sio
$ echo 1a86 5512 | sudo tee /sys/bus/usb-serial/drivers/generic/new_id
```

* On Linux you must run the tool with `sudo`, unless you are using special udev rules (see below).

#### Using the tool without sudo

If you create `/etc/udev/rules.d/80-ch341a.rules` with these lines:
```
# CH341A
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="5512", MODE="0666", TAG+="uaccess"
```

...then you can run `ch341tool` without root privileges.

### Usage

`ch341tool [options] commands...`

#### Commands

- read chip ID: `read_id`.

- reading: `read addr size filename`.

- erasing: `erase addr size cmd step addr_bits`, `cmd` is the erase command code, `step` is the block size for the specified command, `addr_bits` is the address length (24 or 32).

- chip erase: `chip_erase cmd`, `cmd` is the erase command code (usually `0x60` or `0xc7`).

- writing: `write addr size filename file_offset`, use `page_size N` command (before the write command) to set the page size (default 256).

- Release Deep Power-down / Read Electronic Signature: `rdp_res`, could help wake up some chips.

#### Commands (SD cards)

These extra commands are for SD cards, which can be connected to the CH341 using a little board with an SD card slot (often sold as modules for Arduino).

- `sd_init`: initialize the SD card.

- `sd_info`: print info about the SD card.

References: [docs](https://www.sdcard.org/downloads/pls/), [arduino library](https://github.com/arduino-libraries/SD)

#### Commands (RC522 RFID module)

- `rd522_init`: initialize the RFID module.

- `rd522_test`: test the RFID module.

The commands below are only available if you clone [Arduino_MFRC522v2](https://github.com/OSSLibraries/Arduino_MFRC522v2) into the `Arduino/Arduino_MFRC522v2` directory, and build the tool with `MFRC522v2=1`.

- `rc522_wait N`: wait for the RFID card for N seconds.

- `rc522_dump`: dump the RFID card.

- `rc522_key`: set the key for Mifare Classic (6 hex bytes), for example `rc522_key ff-ff-ff-ff-ff-ff`.

- `rc522_dump2`: dump the Mifare Classic card, using the key specified with `rc522_key`.

References: [docs](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf), [arduino library](https://github.com/OSSLibraries/Arduino_MFRC522v2), [simple arduino library](https://github.com/miguelbalboa/rfid)

#### Commands (PN532 RFID module)

- `pn532_init`: initialize the RFID module.

References: [arduino library](https://github.com/Seeed-Studio/PN532)

