
enum {
	PN532_PREAMBLE    = 0x00,
	PN532_STARTCODE1  = 0x00,
	PN532_STARTCODE2  = 0xff,
	PN532_POSTAMBLE   = 0x00,
	PN532_HOSTTOPN532 = 0xd4,
	PN532_PN532TOHOST = 0xd5,

	PN532_COMMAND_DIAGNOSE              = 0x00,
	PN532_COMMAND_GETFIRMWAREVERSION    = 0x02,
	PN532_COMMAND_GETGENERALSTATUS      = 0x04,
	PN532_COMMAND_READREGISTER          = 0x06,
	PN532_COMMAND_WRITEREGISTER         = 0x08,
	PN532_COMMAND_READGPIO              = 0x0C,
	PN532_COMMAND_WRITEGPIO             = 0x0E,
	PN532_COMMAND_SETSERIALBAUDRATE     = 0x10,
	PN532_COMMAND_SETPARAMETERS         = 0x12,
	PN532_COMMAND_SAMCONFIGURATION      = 0x14,
	PN532_COMMAND_POWERDOWN             = 0x16,
	PN532_COMMAND_RFCONFIGURATION       = 0x32,
	PN532_COMMAND_RFREGULATIONTEST      = 0x58,
	PN532_COMMAND_INJUMPFORDEP          = 0x56,
	PN532_COMMAND_INJUMPFORPSL          = 0x46,
	PN532_COMMAND_INLISTPASSIVETARGET   = 0x4A,
	PN532_COMMAND_INATR                 = 0x50,
	PN532_COMMAND_INPSL                 = 0x4E,
	PN532_COMMAND_INDATAEXCHANGE        = 0x40,
	PN532_COMMAND_INCOMMUNICATETHRU     = 0x42,
	PN532_COMMAND_INDESELECT            = 0x44,
	PN532_COMMAND_INRELEASE             = 0x52,
	PN532_COMMAND_INSELECT              = 0x54,
	PN532_COMMAND_INAUTOPOLL            = 0x60,
	PN532_COMMAND_TGINITASTARGET        = 0x8C,
	PN532_COMMAND_TGSETGENERALBYTES     = 0x92,
	PN532_COMMAND_TGGETDATA             = 0x86,
	PN532_COMMAND_TGSETDATA             = 0x8E,
	PN532_COMMAND_TGSETMETADATA         = 0x94,
	PN532_COMMAND_TGGETINITIATORCOMMAND = 0x88,
	PN532_COMMAND_TGRESPONSETOINITIATOR = 0x90,
	PN532_COMMAND_TGGETTARGETSTATUS     = 0x8A,

	PN532_DATA_WRITE  = 1,
	PN532_STATUS_READ = 2,
	PN532_DATA_READ   = 3,

	PN532_ACK_WAIT_TIME = 10
};

static void pn532_spi_low(usbio_t *io) {
	io->buf[0] = CH341A_CMD_UIO_STREAM;
	io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x36;
	io->buf[2] = CH341A_CMD_UIO_STM_DIR | 0x3f;
	io->buf[3] = CH341A_CMD_UIO_STM_END;
	usb_send(io, NULL, 4);
}

static void pn532_spi_high(usbio_t *io) {
	io->buf[0] = CH341A_CMD_UIO_STREAM;
	io->buf[1] = CH341A_CMD_UIO_STM_OUT | 0x37;
	io->buf[2] = CH341A_CMD_UIO_STM_END;
	usb_send(io, NULL, 3);
}

static void pn532_spi_raw(usbio_t *io, const uint8_t *src, uint8_t *out, unsigned len) {
	while (len) {
		int i, n = len;
		if (n > CH341_PACKET_LENGTH - 1)
			n = CH341_PACKET_LENGTH - 1;
		len -= n;

		io->buf[0] = CH341A_CMD_SPI_STREAM;
		for (i = 0; i < n; i++)
			io->buf[1 + i] = src ? *src++ : 0xff;
		if (io->verbose == 2) {
			DBG_LOG("send (%d):\n", n);
			print_mem(stderr, io->buf + 1, n);
		}

		usb_send(io, NULL, n + 1);
		if (usb_recv(io, n) != n)
			ERR_EXIT("unexpected recv size\n");

		if (out)
		for (i = 0; i < n; i++) *out++ = io->buf[i];
		if (io->verbose == 2) {
			DBG_LOG("recv (%d):\n", n);
			print_mem(stderr, io->buf, n);
		}
	}
}

static int pn532_command = 0;

static void pn532_wakeup(usbio_t *io) {
	pn532_spi_low(io);
	usleep(2 * 1000);
	pn532_spi_high(io);
}

static int pn532_ready(usbio_t *io) {
	uint8_t buf[2] = { PN532_STATUS_READ, 0xff };
	pn532_spi_low(io);
	pn532_spi_raw(io, buf, buf, 2);
	pn532_spi_high(io);
	return buf[1] & 1;
}

static void pn532_wait(usbio_t *io) {
	int i;
	for (i = 0; i < PN532_ACK_WAIT_TIME; i++) {
		if (pn532_ready(io)) return;
		usleep(1 * 1000);
	}
	ERR_EXIT("pn532 wait timeout\n");
}

static int pn532_readack(usbio_t *io) {
	uint8_t buf[6];
	const uint8_t ack[1 + 6] = { PN532_DATA_READ, 0, 0, 0xff, 0, 0xff, 0 };
	pn532_spi_low(io);
	usleep(1 * 1000);
	pn532_spi_raw(io, ack, NULL, 1);
	pn532_spi_raw(io, NULL, buf, 6);
	pn532_spi_high(io);
	return memcmp(buf, ack + 1, 6);
}

static void pn532_writecmd(usbio_t *io,
		const uint8_t *header, int hlen, const uint8_t *body, int blen) {
	uint8_t buf[6];
	int i, length = hlen + blen + 1;
	int sum = PN532_HOSTTOPN532;

	pn532_command = header[0];
	if (io->verbose == 2) {
		DBG_LOG("header (%d):\n", hlen);
		print_mem(stderr, header, hlen);
		if (blen) {
			DBG_LOG("body (%d):\n", blen);
			print_mem(stderr, body, blen);
		}
	}
	for (i = 0; i < hlen; i++) sum += header[i];
	for (i = 0; i < blen; i++) sum += body[i];

	pn532_spi_low(io);
	usleep(2 * 1000);
	buf[0] = PN532_DATA_WRITE;
	pn532_spi_raw(io, buf, NULL, 1);
	buf[0] = PN532_PREAMBLE;
	buf[1] = PN532_STARTCODE1;
	buf[2] = PN532_STARTCODE2;
	buf[3] = length;
	buf[4] = 0u - length;
	buf[5] = PN532_HOSTTOPN532;
	pn532_spi_raw(io, buf, NULL, 6);
	pn532_spi_raw(io, header, NULL, hlen);
	if (blen) pn532_spi_raw(io, body, NULL, blen);
	buf[0] = 0u - sum;
	buf[1] = PN532_POSTAMBLE;
	pn532_spi_raw(io, buf, NULL, 2);
	pn532_spi_high(io);
	pn532_wait(io);
	if (pn532_readack(io))
		ERR_EXIT("pn532 readack failed\n");
}

static int pn532_readresp(usbio_t *io, uint8_t *dst, int len) {
	const uint8_t head[1] = { PN532_DATA_READ };
	uint8_t buf[256];
	int i, len1, len2, sum;

	pn532_wait(io);
	pn532_spi_low(io);
	usleep(1 * 1000);
	pn532_spi_raw(io, head, NULL, 1);
	pn532_spi_raw(io, NULL, buf, 7);
	if (buf[0] != PN532_PREAMBLE ||
			buf[1] != PN532_STARTCODE1 ||
			buf[2] != PN532_STARTCODE2 ||
			buf[5] != PN532_PN532TOHOST)
		ERR_EXIT("pn532 response: wrong preamble\n");
	len2 = buf[3];
	if ((len2 + buf[4]) & 0xff)
		ERR_EXIT("pn532 response: wrong length checksum\n");
	sum = pn532_command + 1;
	if (buf[6] != sum)
		ERR_EXIT("pn532 response: wrong command\n");
	sum += PN532_PN532TOHOST; 
	if (len2 < 2)
		ERR_EXIT("pn532 response: returned length is too short\n");
	len2 -= 2;
	pn532_spi_raw(io, NULL, buf, len2 + 1);
	for (i = 0; i < len2; i++) sum += buf[i];
	if ((buf[len2] + sum) & 0xff)
		ERR_EXIT("pn532 response: wrong checksum\n");
	len1 = len < len2 ? len : len2;
	memcpy(dst, buf, len1);
	if (len > len1)
		memset(dst + len1, 0xff, len - len1);
	pn532_spi_high(io);
	return len2;
}

static void pn532_init(usbio_t *io) {
	uint8_t cmd[1] = { PN532_COMMAND_GETFIRMWAREVERSION };
	uint8_t buf[4];
	int ver = 0;
	pn532_writecmd(io, cmd, 1, NULL, 0);
	pn532_readresp(io, buf, 4);
	ver = buf[0] << 24;
	ver |= buf[1] << 16;
	ver |= buf[2] << 8;
	ver |= buf[3];
	DBG_LOG("pn532 version: 0x%08x\n", ver);
}

