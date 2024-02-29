/*
// ch341 programmer for Linux.
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
	if (io->verbose >= 2) {
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

			if (io->verbose >= 2) {
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

		io->buf[0] = CH341A_CMD_SPI_STREAM;
		for (i = 0; i < n; i++)
			io->buf[1 + i] = src ? swap_bits(*src++) : 0xff;

		usb_send(io, NULL, n + 1);
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

static void ch341_sd_wait(usbio_t *io) {
	uint8_t buf[1]; int i;

	for (i = 0; i < 1000; i++) {
		ch341_spi_stream(io, NULL, buf, 1);
		if (buf[0] == 0xff) break;
		usleep(1 * 1000);
	}
	if (buf[0] != 0xff)
		ERR_EXIT("sd_wait failed (ret = 0x%02x)\n", buf[0]);
}

static int sd_crc7(const uint8_t *s, unsigned n) {
	unsigned j, crc = 0;
	while (n--) {
		crc ^= *s++;
#if 1 // should be faster than loop
		// do 7 bits at a time
		j = (crc ^ crc >> 4) & 0xfe;
		crc = (crc << 7 ^ j ^ j << 3) & 0xfe;
		// one more bit
		j = crc >> 7;
		crc = (crc ^ j ^ j << 3) << 1 & 0xfe;
#else
		for (j = 0; j < 8; j++) {
			if (crc & 0x80) crc ^= 0x89;
			crc <<= 1;
		}
#endif
	}
	return crc | 1;
}

static int sd_crc16(const uint8_t *s, unsigned n) {
	unsigned t, crc = 0;
	while (n--) {
		t = crc >> 8 ^ *s++;
		t ^= t >> 4;
		crc = (crc << 8 ^ t ^ t << 5 ^ t << 12) & 0xffff;
	}
	return crc;
}

static int ch341_sd_cmd(usbio_t *io, unsigned cmd, unsigned arg) {
	uint8_t buf[6]; unsigned n;

	ch341_sd_wait(io);

	buf[0] = cmd | 0x40;
	buf[1] = arg >> 24;
	buf[2] = arg >> 16;
	buf[3] = arg >> 8;
	buf[4] = arg;
	buf[5] = sd_crc7(buf, 5);
	// buf[5] = cmd == 0 ? 0x95 : cmd == 8 ? 0x87 : 0xff;

	ch341_spi_stream(io, buf, buf, sizeof(buf));

	for (n = 1000; n; n--) {
		ch341_spi_stream(io, NULL, buf, 1);
		if (!(buf[0] & 0x80)) break;
		usleep(1 * 1000);
	}
	return buf[0];
}

static int ch341_sd_acmd(usbio_t *io, unsigned cmd, unsigned arg) {
	ch341_sd_cmd(io, 55, 0); // APP_CMD
	return ch341_sd_cmd(io, cmd, arg);
}

static void ch341_sd_reg(usbio_t *io, unsigned cmd, void *out, unsigned n) {
	uint8_t buf[64 + 2];
	int ret, i, crc;

	if (cmd == 13 || cmd == 51) // SD_STATUS or SEND_SCR
		ch341_sd_cmd(io, 55, 0); // APP_CMD
	ret = ch341_sd_cmd(io, cmd, 0);
	if (cmd == 13) { // SD_STATUS
		ch341_spi_stream(io, NULL, buf, 1);
		ret = ret << 8 | buf[0];
	}
	if (ret != 0)
		ERR_EXIT("sd(CMD%u) failed (ret = 0x%02x)\n", cmd, ret);
	for (i = 0; i < 1000; i++) {
		ch341_spi_stream(io, NULL, buf, 1);
		ret = buf[0];
		if (ret != 0xff) break;
		usleep(1 * 1000);
	}
	if (ret != 0xfe)
		ERR_EXIT("sd(CMD%u) failed (ret = 0x%02x)\n", cmd, ret);
	ch341_spi_stream(io, NULL, buf, n + 2);
	crc = buf[n] << 8 | buf[n + 1];
	if (crc != sd_crc16(buf, n))
		ERR_EXIT("sd(CMD%u) bad checksum (crc = 0x%04x)\n", cmd, crc);
	memcpy(out, buf, n);
}

static void sd_print_reg(const char *name, const uint8_t *buf, int n) {
	int i;
	DBG_LOG("%s: ", name);
	for (i = 0; i < n; i++) DBG_LOG("%02x%s", buf[i],
			i + 1 == n ? "\n" : (i + 1) & 15 ? " " : "\n     ");
}

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
			int ret, sd_ver = 0, i;
			uint8_t buf[4];
			ret = ch341_sd_cmd(io, 0, 0); // GO_IDLE_STATE
			if (ret != 1) // 1 : idle state 
				ERR_EXIT("sd(CMD0) failed (ret = 0x%02x)\n", ret);
			ret = ch341_sd_cmd(io, 8, 0x1aa); // SEND_IF_COND
			if (!(ret & 4)) { // 4 : illegal command
				ch341_spi_stream(io, NULL, buf, 4);
				if (buf[3] != 0xaa)
					ERR_EXIT("sd(CMD8) failed\n");
				sd_ver = 1;
			}

			i = 0;
			do {
				// SD_APP_OP_COND
				ret = ch341_sd_acmd(io, 41, sd_ver << 30);
				if (ret == 0) break;
				usleep(1 * 1000);
			} while (++i < 1000);
			if (ret != 0)
				ERR_EXIT("sd(ACMD41) failed (ret = 0x%02x)\n", ret);

			ret = ch341_sd_cmd(io, 58, 0); // READ_OCR
			if (ret != 0)
				ERR_EXIT("sd(CMD58) failed (ret = 0x%02x)\n", ret);
			ch341_spi_stream(io, NULL, buf, 4);
			if (io->verbose == 1) sd_print_reg("OCR", buf, 4);
			// 31 : power up status
			// 30 : Card Capacity Status (CCS)
			if (buf[0] >> 6 == 3) {
				sd_ver = 3;
				DBG_LOG("SD V%u (UHS-%s)\n", sd_ver, "II" + !(buf[0] & 0x20));
			} else {
				DBG_LOG("SD V%u\n", sd_ver);
			}
			argc -= 1; argv += 1;

		} else if (!strcmp(argv[1], "sd_info")) {
			uint8_t buf[64]; char name[5];
			unsigned csd_ver, read_bl_len;
			unsigned c_size, c_size_mult;
			int i, ret; uint32_t tmp;
			static const char * const sd_au_size[16] = {
					"not defined", "16 KB", "32 KB", "64 KB",
					"128 KB", "256 KB", "512 KB", "1 MB",
					"2 MB", "4 MB", "8 MB", "12 MB",
					"16 MB", "24 MB", "32 MB", "64 MB" };

			ch341_sd_reg(io, 51, buf, 8); // SEND_SCR
			if (io->verbose == 1) sd_print_reg("SCR", buf, 8);

			DBG_LOG("physical layer version: ");
			tmp = (buf[2] << 8 | buf[3]) & 0x87c0;
			{
				const char *s = NULL;
				int sd_spec = buf[0] & 15;
				int sd_specx = tmp >> 6 & 15;

				if (!tmp) {
					if (sd_spec == 0) s = "1.01";
					else if (sd_spec == 1) s = "1.10";
					else if (sd_spec == 2) s = "2.00";
				} else if (sd_spec == 2 && (tmp & 0x8000)) {
					if (tmp == 0x8000) s = "3.0X";
					else { s = ""; DBG_LOG("%u.XX", 4 + sd_specx); }
				}
				if (s) DBG_LOG("%s\n", s);
				else DBG_LOG("unknown (spec = %u, spec3 = %u, spec4 = %u, specx = %u)\n",
						sd_spec, tmp >> 15, tmp >> 10 & 1, sd_specx);
			}

			ch341_sd_reg(io, 9, buf, 16); // SEND_CSD
			if (io->verbose == 1) sd_print_reg("CSD", buf, 16);
			csd_ver = buf[0] >> 6;
			read_bl_len = buf[5] & 0xf;
			if (csd_ver == 0) {
				if ((read_bl_len - 9) >= 3) // valid : 9, 10, 11
					ERR_EXIT("invalid READ_BL_LEN (%u)\n", read_bl_len);
				c_size = buf[6] << 16 | buf[7] << 8 | buf[8];
				c_size = c_size >> 6 & 0xfff;
				c_size_mult = (buf[9] << 8 | buf[10]) >> 7 & 7;
			} else if (csd_ver == 1 || csd_ver == 2) {
				if (read_bl_len != 9)
					ERR_EXIT("invalid READ_BL_LEN (%u)\n", read_bl_len);
				c_size = buf[6] << 24 | buf[7] << 16 | buf[8] << 8 | buf[9];
				c_size &= 0xfffffff;
				if (csd_ver == 1) c_size &= 0x3fffff;
				c_size_mult = 10;
			} else
				DBG_LOG("unknown CSD version (%u)\n", ret);

			if (csd_ver < 3) {
				unsigned long long sd_size = c_size + 1;
				sd_size <<= c_size_mult + read_bl_len;
				DBG_LOG("device size: %llu\n", sd_size);
			}

			ch341_sd_reg(io, 10, buf, 16); // SEND_CID
			if (io->verbose == 1) sd_print_reg("CID", buf, 16);

			tmp = buf[0];
			DBG_LOG("Manufacturer ID: %u (0x%02x)\n", tmp, tmp);
			tmp = buf[1] << 8 | buf[2];

			for (i = 0; i < 2; i++) {
				int a = buf[1 + i];
				name[i] = a >= 0x20 && a < 0x7f ? a : '?';
			}
			DBG_LOG("OEM/Application ID: \"%.2s\" (%02x %02x)\n", name, buf[1], buf[2]);

			for (i = 0; i < 5; i++) {
				int a = buf[3 + i];
				name[i] = a >= 0x20 && a < 0x7f ? a : '?';
			}
			DBG_LOG("name: \"%.5s\" (%02x %02x %02x %02x %02x)\n",
					name, buf[3], buf[4], buf[5], buf[6], buf[7]);
			tmp = buf[8];
			DBG_LOG("revision: %u.%u\n", tmp >> 4, tmp & 15);
			tmp = buf[9] << 24 | buf[10] << 16 | buf[11] << 8 | buf[12];
			DBG_LOG("serial number: 0x%08x\n", tmp);
			tmp = (buf[13] << 8 | buf[14]) & 0xfff;
			DBG_LOG("manufacturing date: %u.%02u\n", (tmp >> 4) + 2000, tmp & 15);

			ch341_sd_reg(io, 13, buf, 64); // SD_STATUS
			if (io->verbose == 1) sd_print_reg("SSR", buf, 64);

			tmp = buf[8];
			if (tmp < 5) {
				if (tmp >= 4) tmp++;
				DBG_LOG("speed class: %u\n", tmp * 2);
			} else {
				DBG_LOG("speed class: unknown (%u)\n", tmp);
			}
			tmp = buf[10] >> 4;
			DBG_LOG("AU size: %s\n", sd_au_size[tmp]);

			tmp = buf[14] >> 4;
			{
				const char *grade = NULL;
				switch (tmp) {
				case 0: grade = "less than 10MB/sec"; break;
				case 1: grade = "10MB/sec and above"; break;
				case 3: grade = "30MB/sec and above"; break;
				}
				if (grade) DBG_LOG("UHS speed grade: %s\n", grade);
				else DBG_LOG("UHS speed grade: unknown (%u)\n", tmp);
			}
			tmp = buf[14] & 15;
			DBG_LOG("UHS AU size: %s\n", sd_au_size[tmp]);
			DBG_LOG("video speed class: %u\n", buf[15]);

			DBG_LOG("VSC AU size: ");
			tmp = (buf[16] << 8 | buf[17]) & 0x3ff;
			if (!tmp) DBG_LOG("not supported\n");
			else DBG_LOG("%u MB\n", tmp);

			tmp = buf[21] & 15;
			DBG_LOG("application performance class: ");
			if (!tmp) DBG_LOG("not supported\n");
			else DBG_LOG(tmp < 3 ? "A%u\n" : "unknown (%u)", tmp);
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
