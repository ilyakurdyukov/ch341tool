/*
// CH341A/B programmer tool for Linux.
//
// sudo modprobe ftdi_sio
// echo 1a86 5512 | sudo tee /sys/bus/usb-serial/drivers/generic/new_id
// make && sudo ./ch341prog [options] commands...
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
*/

#define _GNU_SOURCE 1

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifndef LIBUSB_DETACH
/* detach the device from crappy kernel drivers */
#define LIBUSB_DETACH 1
#endif

#if USE_LIBUSB
#include <libusb-1.0/libusb.h>
#else
#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#endif
#include <unistd.h>

static void print_mem(FILE *f, const uint8_t *buf, size_t len) {
	size_t i; int a, j, n;
	for (i = 0; i < len; i += 16) {
		n = len - i;
		if (n > 16) n = 16;
		for (j = 0; j < n; j++) fprintf(f, "%02x ", buf[i + j]);
		for (; j < 16; j++) fprintf(f, "   ");
		fprintf(f, " |");
		for (j = 0; j < n; j++) {
			a = buf[i + j];
			fprintf(f, "%c", a > 0x20 && a < 0x7f ? a : '.');
		}
		fprintf(f, "|\n");
	}
}

#define ERR_EXIT(...) \
	do { fprintf(stderr, __VA_ARGS__); exit(1); } while (0)

#define DBG_LOG(...) fprintf(stderr, __VA_ARGS__)

#define RECV_BUF_LEN 1024
#define TEMP_BUF_LEN 0x1000

typedef struct {
	uint8_t *recv_buf, *buf;
#if USE_LIBUSB
	libusb_device_handle *dev_handle;
	int endp_in, endp_out;
#else
	int serial;
#endif
	int flags, recv_len, recv_pos, nread;
	int verbose, timeout;
} usbio_t;

#if USE_LIBUSB
static void find_endpoints(libusb_device_handle *dev_handle, int result[2]) {
	int endp_in = -1, endp_out = -1;
	int i, k, err;
	//struct libusb_device_descriptor desc;
	struct libusb_config_descriptor *config;
	libusb_device *device = libusb_get_device(dev_handle);
	if (!device)
		ERR_EXIT("libusb_get_device failed\n");
	//if (libusb_get_device_descriptor(device, &desc) < 0)
	//	ERR_EXIT("libusb_get_device_descriptor failed");
	err = libusb_get_config_descriptor(device, 0, &config);
	if (err < 0)
		ERR_EXIT("libusb_get_config_descriptor failed : %s\n", libusb_error_name(err));

	for (k = 0; k < config->bNumInterfaces; k++) {
		const struct libusb_interface *interface;
		const struct libusb_interface_descriptor *interface_desc;
		int claim = 0;
		interface = config->interface + k;
		if (interface->num_altsetting < 1) continue;
		interface_desc = interface->altsetting + 0;
		for (i = 0; i < interface_desc->bNumEndpoints; i++) {
			const struct libusb_endpoint_descriptor *endpoint;
			endpoint = interface_desc->endpoint + i;
			if (endpoint->bmAttributes == 2) {
				int addr = endpoint->bEndpointAddress;
				err = 0;
				if (addr & 0x80) {
					if (endp_in >= 0) ERR_EXIT("more than one endp_in\n");
					endp_in = addr;
					claim = 1;
				} else {
					if (endp_out >= 0) ERR_EXIT("more than one endp_out\n");
					endp_out = addr;
					claim = 1;
				}
			}
		}
		if (claim) {
			i = interface_desc->bInterfaceNumber;
#if LIBUSB_DETACH
			err = libusb_kernel_driver_active(dev_handle, i);
			if (err > 0) {
				DBG_LOG("kernel driver is active, trying to detach\n");
				err = libusb_detach_kernel_driver(dev_handle, i);
				if (err < 0)
					ERR_EXIT("libusb_detach_kernel_driver failed : %s\n", libusb_error_name(err));
			}
#endif
			err = libusb_claim_interface(dev_handle, i);
			if (err < 0)
				ERR_EXIT("libusb_claim_interface failed : %s\n", libusb_error_name(err));
			break;
		}
	}
	if (endp_in < 0) ERR_EXIT("endp_in not found\n");
	if (endp_out < 0) ERR_EXIT("endp_out not found\n");
	libusb_free_config_descriptor(config);

	//DBG_LOG("USB endp_in=%02x, endp_out=%02x\n", endp_in, endp_out);

	result[0] = endp_in;
	result[1] = endp_out;
}
#else
static void init_serial(int serial) {
	struct termios tty = { 0 };

	// B921600
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	tty.c_cflag = CS8 | CLOCAL | CREAD;
	tty.c_iflag = IGNPAR;
	tty.c_oflag = 0;
	tty.c_lflag = 0;

	tty.c_cc[VMIN] = 1;
	tty.c_cc[VTIME] = 0;

	tcflush(serial, TCIFLUSH);
	tcsetattr(serial, TCSANOW, &tty);
}
#endif

#if USE_LIBUSB
static usbio_t* usbio_init(libusb_device_handle *dev_handle, int flags) {
#else
static usbio_t* usbio_init(int serial, int flags) {
#endif
	uint8_t *p; usbio_t *io;

#if USE_LIBUSB
	int endpoints[2];
	find_endpoints(dev_handle, endpoints);
#else
	init_serial(serial);
	// fcntl(serial, F_SETFL, FNDELAY);
	tcflush(serial, TCIOFLUSH);
#endif

	p = (uint8_t*)malloc(sizeof(usbio_t) + RECV_BUF_LEN + TEMP_BUF_LEN);
	io = (usbio_t*)p; p += sizeof(usbio_t);
	if (!p) ERR_EXIT("malloc failed\n");
	io->flags = flags;
#if USE_LIBUSB
	io->dev_handle = dev_handle;
	io->endp_in = endpoints[0];
	io->endp_out = endpoints[1];
#else
	io->serial = serial;
#endif
	io->recv_len = 0;
	io->recv_pos = 0;
	io->recv_buf = p; p += RECV_BUF_LEN;
	io->buf = p;
	io->verbose = 0;
	io->timeout = 1000;
	return io;
}

static void usbio_free(usbio_t* io) {
	if (!io) return;
#if USE_LIBUSB
	libusb_close(io->dev_handle);
#else
	close(io->serial);
#endif
	free(io);
}

#define WRITE16_BE(p, a) do { \
	uint32_t __tmp = a; \
	((uint8_t*)(p))[0] = (uint8_t)(__tmp >> 8); \
	((uint8_t*)(p))[1] = (uint8_t)(a); \
} while (0)

#define WRITE32_LE(p, a) do { \
	uint32_t __tmp = a; \
	((uint8_t*)(p))[0] = (uint8_t)__tmp; \
	((uint8_t*)(p))[1] = (uint8_t)(__tmp >> 8); \
	((uint8_t*)(p))[2] = (uint8_t)(__tmp >> 16); \
	((uint8_t*)(p))[3] = (uint8_t)(__tmp >> 24); \
} while (0)

#define READ32_BE(p) ( \
	((uint8_t*)(p))[0] << 24 | \
	((uint8_t*)(p))[1] << 16 | \
	((uint8_t*)(p))[2] << 8 | \
	((uint8_t*)(p))[3])

#define READ32_LE(p) ( \
	((uint8_t*)(p))[3] << 24 | \
	((uint8_t*)(p))[2] << 16 | \
	((uint8_t*)(p))[1] << 8 | \
	((uint8_t*)(p))[0])

static int usb_send(usbio_t *io, const void *data, int len) {
	const uint8_t *buf = (const uint8_t*)data;
	int ret;

	if (!buf) buf = io->buf;
	if (!len) ERR_EXIT("empty message\n");
	if (io->verbose >= 3) {
		DBG_LOG("send (%d):\n", len);
		print_mem(stderr, buf, len);
	}

#if USE_LIBUSB
	{
		int err = libusb_bulk_transfer(io->dev_handle,
				io->endp_out, (uint8_t*)buf, len, &ret, io->timeout);
		if (err < 0)
			ERR_EXIT("usb_send failed : %s\n", libusb_error_name(err));
	}
#else
	ret = write(io->serial, buf, len);
#endif
	if (ret != len)
		ERR_EXIT("usb_send failed (%d / %d)\n", ret, len);

#if !USE_LIBUSB
	tcdrain(io->serial);
	// usleep(1000);
#endif
	return ret;
}

static int usb_recv(usbio_t *io, int plen) {
	int a, pos, len, nread = 0;
	if (plen > TEMP_BUF_LEN)
		ERR_EXIT("target length too long\n");

	len = io->recv_len;
	pos = io->recv_pos;
	while (nread < plen) {
		if (pos >= len) {
#if USE_LIBUSB
			int err = libusb_bulk_transfer(io->dev_handle, io->endp_in, io->recv_buf, RECV_BUF_LEN, &len, io->timeout);
			if (err == LIBUSB_ERROR_NO_DEVICE)
				ERR_EXIT("connection closed\n");
			else if (err == LIBUSB_ERROR_TIMEOUT) break;
			else if (err < 0)
				ERR_EXIT("usb_recv failed : %s\n", libusb_error_name(err));
#else
			if (io->timeout >= 0) {
				struct pollfd fds = { 0 };
				fds.fd = io->serial;
				fds.events = POLLIN;
				a = poll(&fds, 1, io->timeout);
				if (a < 0) ERR_EXIT("poll failed, ret = %d\n", a);
				if (fds.revents & POLLHUP)
					ERR_EXIT("connection closed\n");
				if (!a) break;
			}
			len = read(io->serial, io->recv_buf, RECV_BUF_LEN);
#endif
			if (len < 0)
				ERR_EXIT("usb_recv failed, ret = %d\n", len);

			if (io->verbose >= 3) {
				DBG_LOG("recv (%d):\n", len);
				print_mem(stderr, io->recv_buf, len);
			}
			pos = 0;
			if (!len) break;
		}
		a = io->recv_buf[pos++];
		io->buf[nread++] = a;
	}
	io->recv_len = len;
	io->recv_pos = pos;
	io->nread = nread;
	return nread;
}

#include "ch341a.h"

static void ch341_i2c_set(usbio_t *io, int set) {
	io->buf[0] = CH341A_CMD_I2C_STREAM;
	io->buf[1] = CH341A_CMD_I2C_STM_SET | set;
	io->buf[2] = CH341A_CMD_I2C_STM_END;
	usb_send(io, NULL, 3);
}

static inline unsigned swap_bits(unsigned a) {
	a = (a & 0x55) << 1 | (a & 0xaa) >> 1;
	a = (a & 0x33) << 2 | (a & 0xcc) >> 2;
	a = (a & 0x0f) << 4 | (a & 0xf0) >> 4;
	return a;
}

static void ch341_spi_stream(usbio_t *io, const uint8_t *src, uint8_t *out, unsigned len) {
	io->buf[0] = CH341A_CMD_UIO_STREAM;
	io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x36;
	io->buf[2] = CH341A_CMD_UIO_STM_DIR | 0x3f;
	io->buf[3] = CH341A_CMD_UIO_STM_END;
	usb_send(io, NULL, 4);

	while (len) {
		int i, n = len;
		if (n > CH341_PACKET_LENGTH - 1)
			n = CH341_PACKET_LENGTH - 1;
		len -= n;

		if (io->verbose == 2) {
			if (!src) memset(io->buf, 0xff, n);
			DBG_LOG("send (%d):\n", n);
			print_mem(stderr, src ? src : io->buf, n);
		}
		io->buf[0] = CH341A_CMD_SPI_STREAM;
		for (i = 0; i < n; i++)
			io->buf[1 + i] = src ? swap_bits(*src++) : 0xff;

		usb_send(io, NULL, n + 1);
		if (usb_recv(io, n) != n)
			ERR_EXIT("unexpected recv size\n");

		if (out)
		for (i = 0; i < n; i++)
			*out++ = swap_bits(io->buf[i]);
		if (io->verbose == 2) {
			if (!out)
			for (i = 0; i < n; i++)
				io->buf[i] = swap_bits(io->buf[i]);
			DBG_LOG("recv (%d):\n", n);
			print_mem(stderr, out ? out - n : io->buf, n);
		}
	}

	io->buf[0] = CH341A_CMD_UIO_STREAM;
	io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
	io->buf[2] = CH341A_CMD_UIO_STM_END;
	usb_send(io, NULL, 3);
}

#if 0
// AsProgrammer style, found using USB capture. Is this more correct?
static void ch341_spi_stream2(usbio_t *io, const uint8_t *src, uint8_t *out, unsigned len1, unsigned len2) {
	int i, n;
	if (len1) {
		io->buf[0] = CH341A_CMD_UIO_STREAM;
		io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0;
		io->buf[2] = CH341A_CMD_UIO_STM_DIR | 0x29; // 0b101001
		io->buf[3] = CH341A_CMD_UIO_STM_END;
		usb_send(io, NULL, 4);

		n = len1;
		io->buf[0] = CH341A_CMD_SPI_STREAM;
		for (i = 0; i < n; i++)
			io->buf[1 + i] = swap_bits(*src++);

		usb_send(io, NULL, n + 1);
		if (usb_recv(io, n) != n)
			ERR_EXIT("unexpected recv size\n");
		for (i = 0; i < n; i++)
			*out++ = swap_bits(io->buf[i]);
	}

	if (len2) {
		io->buf[0] = CH341A_CMD_UIO_STREAM;
		io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x36;
		io->buf[2] = CH341A_CMD_UIO_STM_DIR | 0x3f;
		io->buf[3] = CH341A_CMD_UIO_STM_END;
		memset(io->buf + 4, 0, 28);
		io->buf[32] = CH341A_CMD_SPI_STREAM;
		n = len2;
		for (i = 0; i < n; i++)
			io->buf[33 + i] = swap_bits(*src++);
		usb_send(io, NULL, 33 + n);
		if (usb_recv(io, n) != n)
			ERR_EXIT("unexpected recv size\n");
		for (i = 0; i < n; i++)
			*out++ = swap_bits(io->buf[i]);
	}

	io->buf[0] = CH341A_CMD_UIO_STREAM;
	io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
	io->buf[2] = CH341A_CMD_UIO_STM_END;
	usb_send(io, NULL, 3);
}
#endif

#define MAX_BLK_SIZE 0x1f00

static void spi_read(usbio_t *io, uint32_t addr, uint32_t len,
		const char *fn, unsigned step) {
	uint8_t buf[5 + MAX_BLK_SIZE];

	FILE *fo = fopen(fn, "wb");
	if (!fo) ERR_EXIT("fopen(wb) failed\n");

	while (len) {
		unsigned pos = 1, n = addr >> 24;
		n = step;
		if (n > len) n = len;
		buf[0] = 0x03; // Read Array
		if (addr + n > 1 << 24) {
			buf[0] = 0x13;
			buf[pos++] = addr >> 24;
		}
		buf[pos++] = addr >> 16;
		buf[pos++] = addr >> 8;
		buf[pos++] = addr;
		memset(buf + pos, 0xff, n);
		ch341_spi_stream(io, buf, buf, pos + n);
		addr += n; len -= n;

		if (fwrite(buf + pos, 1, n, fo) != n) 
			ERR_EXIT("fwrite failed\n");
	}
	fclose(fo);
}

static void wait_write(usbio_t *io, int us) {
	uint8_t buf[2];
	do {
		usleep(us);
		buf[0] = 0x05; // Read Status Register
		buf[1] = 0;
		ch341_spi_stream(io, buf, buf, 2);
	} while (buf[1] & 1);
}

static void spi_write(usbio_t *io, uint32_t addr, uint32_t len,
		const char *fn, size_t offset, unsigned page_size) {
	uint8_t buf[5 + MAX_BLK_SIZE];
	FILE *fi;
	if (page_size > MAX_BLK_SIZE)
		ERR_EXIT("page size is too big\n");
	if ((addr | len) & (page_size - 1))
		ERR_EXIT("address and size must be page aligned\n");

	fi = fopen(fn, "rb");
	if (!fi) ERR_EXIT("fopen(rb) failed\n");
	fseek(fi, offset, SEEK_SET);

	while (len) {
		unsigned i, pos = 1, n;

		// must be done for every page
		buf[0] = 0x06; // Write Enable
		ch341_spi_stream(io, buf, buf, 1);

		n = page_size;
		if (n > len) n = len;
		buf[0] = 0x02; // Page Program
		if (addr >> 24) {
			buf[0] = 0x12;
			buf[pos++] = addr >> 24;
		}
		buf[pos++] = addr >> 16;
		buf[pos++] = addr >> 8;
		buf[pos++] = addr;
		if (fread(buf + pos, 1, n, fi) != n) 
			ERR_EXIT("fread failed\n");
		// skip pages filled with 0xff
		for (i = 0; i < n; i++)
			if (buf[pos + i] != 0xff) break;
		if (i < n) {
			ch341_spi_stream(io, buf, buf, pos + n);
			wait_write(io, 1 * 1000);
		}
		addr += n; len -= n;
	}
	buf[0] = 0x04; // Write Disable
	ch341_spi_stream(io, buf, buf, 1);

	fclose(fi);
}

static void spi_erase(usbio_t *io, uint32_t addr, uint32_t len,
	unsigned cmd, unsigned step, unsigned is32bit) {
	uint8_t buf[4];

	if ((addr | len) & (step - 1))
		ERR_EXIT("address and size must be aligned\n");

	if ((addr + len) >> 24 && !is32bit)
		ERR_EXIT("need 32-bit erase\n");

	while (len) {
		unsigned pos = 1, n;

		buf[0] = 0x06; // Write Enable
		ch341_spi_stream(io, buf, buf, 1);

		buf[0] = cmd;
		if (is32bit)
			buf[pos++] = addr >> 24;
		buf[pos++] = addr >> 16;
		buf[pos++] = addr >> 8;
		buf[pos++] = addr;
		ch341_spi_stream(io, buf, buf, pos);
		wait_write(io, 1 * 1000);
		n = step; addr += n; len -= n;
	}

	buf[0] = 0x04; // Write Disable
	ch341_spi_stream(io, buf, buf, 1);
}

#include "sdcard.h"
#include "rc522.h"
#include "pn532.h"

static uint64_t str_to_size(const char *str) {
	char *end; int shl = 0; uint64_t n;
	n = strtoull(str, &end, 0);
	if (*end) {
		if (!strcmp(end, "K")) shl = 10;
		else if (!strcmp(end, "M")) shl = 20;
		else if (!strcmp(end, "G")) shl = 30;
		else ERR_EXIT("unknown size suffix\n");
	}
	if (shl) {
		int64_t tmp = n;
		tmp >>= 63 - shl;
		if (tmp && ~tmp)
			ERR_EXIT("size overflow on multiply\n");
	}
	return n << shl;
}

static int ch2hex(unsigned a) {
	const char *tab = "abcdef0123456789ABCDEF";
	const char *p = strchr(tab, a);
	return p ? (p - tab + 10) & 15 : -1;
}

#define REOPEN_FREQ 2

int main(int argc, char **argv) {
#if USE_LIBUSB
	libusb_device_handle *device;
#else
	int serial;
#endif
	usbio_t *io; int ret, i;
	int wait = 30 * REOPEN_FREQ;
	const char *tty = "/dev/ttyUSB0";
	int verbose = 0;
	int speed = 0, blk_size = 0x1000, page_size = 256;

#if USE_LIBUSB
	ret = libusb_init(NULL);
	if (ret < 0)
		ERR_EXIT("libusb_init failed: %s\n", libusb_error_name(ret));
#endif

	while (argc > 1) {
		if (!strcmp(argv[1], "--tty")) {
			if (argc <= 2) ERR_EXIT("bad option\n");
			tty = argv[2];
			argc -= 2; argv += 2;
		} else if (!strcmp(argv[1], "--wait")) {
			if (argc <= 2) ERR_EXIT("bad option\n");
			wait = atoi(argv[2]) * REOPEN_FREQ;
			argc -= 2; argv += 2;
		} else if (!strcmp(argv[1], "--speed")) {
			if (argc <= 2) ERR_EXIT("bad option\n");
			speed = atoi(argv[2]) & 7;
			argc -= 2; argv += 2;
		} else if (!strcmp(argv[1], "--verbose")) {
			if (argc <= 2) ERR_EXIT("bad option\n");
			verbose = atoi(argv[2]);
			argc -= 2; argv += 2;
		} else if (argv[1][0] == '-') {
			ERR_EXIT("unknown option\n");
		} else break;
	}

	for (i = 0; ; i++) {
#if USE_LIBUSB
		(void)tty;
		device = libusb_open_device_with_vid_pid(NULL, 0x1a86, 0x5512);
		if (device) break;
		if (i >= wait)
			ERR_EXIT("libusb_open_device failed\n");
#else
		(void)ret;
		serial = open(tty, O_RDWR | O_NOCTTY | O_SYNC);
		if (serial >= 0) break;
		if (i >= wait)
			ERR_EXIT("open(ttyUSB) failed\n");
#endif
		if (!i) DBG_LOG("Waiting for connection (%ds)\n", wait / REOPEN_FREQ);
		usleep(1000000 / REOPEN_FREQ);
	}

#if USE_LIBUSB
	io = usbio_init(device, 0);
#else
	io = usbio_init(serial, 0);
#endif
	io->verbose = verbose;

	ch341_i2c_set(io, speed);

#if ARDUINO_MFRC522v2
	ch341_io = io;
#endif

	while (argc > 1) {
		if (!strcmp(argv[1], "verbose")) {
			if (argc <= 2) ERR_EXIT("bad command\n");
			io->verbose = atoi(argv[2]);
			argc -= 2; argv += 2;

		} else if (!strcmp(argv[1], "read_id")) {
			uint8_t buf[4] = { 0x9f };
			ch341_spi_stream(io, buf, buf, sizeof(buf));
			DBG_LOG("CHIP_ID (0x9F): %02x %02x %02x\n", buf[1], buf[2], buf[3]);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "rdp_res")) {
			// Release Deep Power-down / Read Electronic Signature
			uint8_t buf[5] = { 0xab, 0, 0, 0 };
			ch341_spi_stream(io, buf, buf, sizeof(buf));
			DBG_LOG("CHIP_ID (0xAB): %02x\n", buf[4]);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "read")) {
			const char *fn; uint64_t addr, size;
			if (argc <= 4) ERR_EXIT("bad command\n");

			addr = str_to_size(argv[2]);
			size = str_to_size(argv[3]);
			fn = argv[4];
			if (!size) ERR_EXIT("zero size\n");
			if ((addr | (size - 1) | (addr + size - 1)) >> 32)
				ERR_EXIT("32-bit limit reached\n");
			spi_read(io, addr, size, fn, blk_size);
			argc -= 4; argv += 4;

		} else if (!strcmp(argv[1], "write")) {
			const char *fn; uint64_t addr, size, offset;
			if (argc <= 5) ERR_EXIT("bad command\n");

			addr = str_to_size(argv[2]);
			size = str_to_size(argv[3]);
			fn = argv[4];
			offset = str_to_size(argv[5]);
			if (!size) ERR_EXIT("zero size\n");
			if ((addr | (size - 1) | (addr + size - 1)) >> 32)
				ERR_EXIT("32-bit limit reached\n");
			spi_write(io, addr, size, fn, offset, page_size);
			argc -= 5; argv += 5;

		} else if (!strcmp(argv[1], "erase")) {
			uint64_t addr, size;
			unsigned cmd, step, bits;
			if (argc <= 6) ERR_EXIT("bad command\n");

			addr = str_to_size(argv[2]);
			size = str_to_size(argv[3]);
			cmd = strtoll(argv[4], NULL, 0);
			step = str_to_size(argv[5]);
			bits = atoi(argv[6]);
			if (!size) ERR_EXIT("zero size\n");
			if ((addr | (size - 1) | (addr + size - 1)) >> 32)
				ERR_EXIT("32-bit limit reached\n");
			if (bits != 24 && bits != 32)
				ERR_EXIT("bits must be 24 or 32\n");
			if (!step) ERR_EXIT("step can't be zero\n");
			if ((step & (step - 1)))
				ERR_EXIT("step must be a power of two\n");
			spi_erase(io, addr, size, cmd, step, bits == 32);
			argc -= 6; argv += 6;

		} else if (!strcmp(argv[1], "chip_erase")) {
			uint8_t buf[1]; unsigned cmd;
			if (argc <= 2) ERR_EXIT("bad command\n");
			cmd = strtoll(argv[2], NULL, 0);
			buf[0] = 0x06; // Write Enable
			ch341_spi_stream(io, buf, buf, 1);
			buf[0] = cmd;
			ch341_spi_stream(io, buf, buf, 1);
			wait_write(io, 10 * 1000);
			buf[0] = 0x04; // Write Disable
			ch341_spi_stream(io, buf, buf, 1);
			argc -= 2; argv += 2;

		} else if (!strcmp(argv[1], "blk_size")) {
			if (argc <= 2) ERR_EXIT("bad command\n");
			blk_size = strtol(argv[2], NULL, 0);
			blk_size = blk_size < 1 ? 1 :
					blk_size > MAX_BLK_SIZE ? MAX_BLK_SIZE : blk_size;
			argc -= 2; argv += 2;

		} else if (!strcmp(argv[1], "page_size")) {
			unsigned n;
			if (argc <= 2) ERR_EXIT("bad command\n");
			n = strtol(argv[2], NULL, 0);
			if (!n) ERR_EXIT("page_size can't be zero\n");
			if ((n & (n - 1)))
				ERR_EXIT("page_size must be a power of two\n");
			page_size = n;
			argc -= 2; argv += 2;

		} else if (!strcmp(argv[1], "sd_init")) {
			sd_init(io);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "sd_info")) {
			sd_info(io);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "rc522_init")) {
			rc522_init(io);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "rc522_test")) {
			rc522_test(io);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "rc522_key")) {
			int i; uint8_t *key = rc522_mifare_key;
			char *p = (char*)argv[2];
			int delim = 0;
			if (argc <= 2) ERR_EXIT("bad command\n");
			for (i = 0; i < 6; i++) {
				int h = ch2hex(*p++), a;
				if (h < 0 || (a = ch2hex(*p++)) < 0) break;
				key[i] = h << 4 | a;
				if (!i && ch2hex(*p) < 0) delim = *p;
				if (i == 5) { if (*p) break; }
				else if (delim && *p++ != delim) break;
			}
			if (i != 6) ERR_EXIT("unexpected key format\n");
			if (io->verbose >= 1)
				DBG_LOG("mifare_key: %02x %02x %02x %02x %02x %02x\n",
						key[0], key[1], key[2], key[3], key[4], key[5]);
			argc -= 2; argv += 2;

#if ARDUINO_MFRC522v2
		} else if (!strcmp(argv[1], "rc522_wait")) {
			int timeout; // in seconds
			if (argc <= 2) ERR_EXIT("bad command\n");
			timeout = atoi(argv[2]);
			Arduino_rc522_wait(timeout);
			argc -= 2; argv += 2;

		} else if (!strcmp(argv[1], "rc522_dump")) {
			Arduino_rc522_dump();
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "rc522_dump2")) {
			Arduino_rc522_dump2(rc522_mifare_key);
			argc -= 1; argv += 1;
#endif

		} else if (!strcmp(argv[1], "pn532_init")) {
			pn532_init(io);
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "timeout")) {
			if (argc <= 2) ERR_EXIT("bad command\n");
			io->timeout = atoi(argv[2]);
			argc -= 2; argv += 2;

		} else {
			ERR_EXIT("unknown command\n");
		}
	}

	usbio_free(io);
#if USE_LIBUSB
	libusb_exit(NULL);
#endif
	return 0;
}
