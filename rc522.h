
#ifndef ARDUINO_MFRC522v2
#define ARDUINO_MFRC522v2 1
#endif

static uint8_t rc522_mifare_key[6];

#if !ARDUINO_MFRC522v2
enum {
	// Command and status
	rc522_CommandReg       = 0x01,
	rc522_ComIrqReg        = 0x04,
	rc522_ErrorReg         = 0x06,
	rc522_FIFODataReg      = 0x09,
	rc522_FIFOLevelReg     = 0x0a,
	rc522_BitFramingReg    = 0x0d,
	// Communication
  rc522_ModeReg          = 0x11,
	rc522_TxModeReg        = 0x12,
	rc522_RxModeReg        = 0x13,
	rc522_TxControlReg     = 0x14,
	rc522_TxASKReg         = 0x15,
	// Configuration
	rc522_ModWidthReg      = 0x24,
	rc522_TModeReg         = 0x2a,
	rc522_TPrescalerReg    = 0x2b,
	rc522_TReloadRegH      = 0x2c,
	rc522_TReloadRegL      = 0x2d,
	// Test
	rc522_AutoTestReg      = 0x36,
	rc522_VersionReg       = 0x37,

	rc522_cmd_Idle         = 0,
	rc522_cmd_Mem          = 1,
	rc522_cmd_GenRandomID  = 2,
	rc522_cmd_CalcCRC      = 3,
	rc522_cmd_Transmit     = 4,
	rc522_cmd_NoCmdChange  = 7,
	rc522_cmd_Receive      = 8,
	rc522_cmd_Transceive   = 12,
	rc522_cmd_MFAuthent    = 14,
	rc522_cmd_SoftReset    = 15,
};

static void rc522_write_reg(usbio_t *io, int reg, int val) {
	uint8_t buf[2] = { reg << 1, val };
	ch341_spi_stream(io, buf, buf, 2);
}

static void rc522_write_buf(usbio_t *io, int reg, const uint8_t *src, unsigned len) {
	uint8_t buf[1 + 25];
	if (!len || len >= sizeof(buf))
		ERR_EXIT("write_buf: unexpected length\n");
	buf[0] = reg << 1;
	memcpy(buf + 1, src, len);
	ch341_spi_stream(io, buf, buf, len + 1);
}

static int rc522_read_reg(usbio_t *io, int reg) {
	uint8_t buf[2] = { 0x80 | reg << 1, 0 };
	ch341_spi_stream(io, buf, buf, 2);
	return buf[1];
}

static void rc522_read_buf(usbio_t *io, int reg, uint8_t *dst, unsigned len) {
	uint8_t buf[1 + 64];
	if (!len || len >= sizeof(buf))
		ERR_EXIT("read_buf: unexpected length\n");
	memset(buf, 0x80 | reg << 1, len);
	buf[len] = 0;
	ch341_spi_stream(io, buf, buf, len + 1);
	memcpy(dst, buf + 1, len);
}

static void rc522_reset(usbio_t *io) {
	int i, reg = 0;

	rc522_write_reg(io, rc522_CommandReg, rc522_cmd_SoftReset);
	for (i = 0; i < 100; i++) {
		usleep(10 * 1000);
		reg = rc522_read_reg(io, rc522_CommandReg);
		if (!(reg & 0x10)) break;
	}
	if (reg & 0x10)
		ERR_EXIT("rc522: soft reset failed\n");
}

static void rc522_init(usbio_t *io) {
	int reg;

	rc522_reset(io);

	reg = rc522_read_reg(io, rc522_VersionReg);
	DBG_LOG("rc522 version: 0x%02x\n", reg);

	rc522_write_reg(io, rc522_TxModeReg, 0);
	rc522_write_reg(io, rc522_RxModeReg, 0);
	rc522_write_reg(io, rc522_ModWidthReg, 0x26);

	// (TPrescaler * 2 + 1) * (TReloadVal + 1) / 13560 = delay in us
	reg = 169;
	rc522_write_reg(io, rc522_TModeReg, 0x80 | reg >> 8);
	rc522_write_reg(io, rc522_TPrescalerReg, reg);
	reg = 1000;
	rc522_write_reg(io, rc522_TReloadRegH, reg >> 8);
	rc522_write_reg(io, rc522_TReloadRegL, reg & 0xff);

	rc522_write_reg(io, rc522_TxASKReg, 0x40);
	rc522_write_reg(io, rc522_ModeReg, 0x3d);

	rc522_write_reg(io, rc522_TxControlReg,
			rc522_read_reg(io, rc522_TxControlReg) | 3);
}

static void rc522_test(usbio_t *io) {
	uint8_t buf[25] = { 0 }, res[64];
	int i, reg = 0;

	rc522_reset(io);

	rc522_write_reg(io, rc522_FIFOLevelReg, 0x80);
	rc522_write_buf(io, rc522_FIFODataReg, buf, sizeof(buf));
	rc522_write_reg(io, rc522_CommandReg, rc522_cmd_Mem);
	rc522_write_reg(io, rc522_AutoTestReg, 9);
	rc522_write_reg(io, rc522_FIFODataReg, 0);
	rc522_write_reg(io, rc522_CommandReg, rc522_cmd_CalcCRC);

	for (i = 0; i < 100; i++) {
		usleep(10 * 1000);
		reg = rc522_read_reg(io, rc522_FIFOLevelReg);
		if (reg >= 64) break;
	}
	if (reg < 64)
		ERR_EXIT("rc522: test timeout\n");

	rc522_write_reg(io, rc522_CommandReg, rc522_cmd_Idle);
	rc522_read_buf(io, rc522_FIFODataReg, res, sizeof(res));
	DBG_LOG("data: ");
	for (i = 0; i < 64; i++) DBG_LOG("%02x%s", res[i],
		i == 63 ? "\n" : (i + 1) & 15 ? " " : "\n      ");
	rc522_write_reg(io, rc522_AutoTestReg, 0);
}
#else
static usbio_t *ch341_io;
void Arduino_spi_stream(const uint8_t *src, uint8_t *out, unsigned len) {
	ch341_spi_stream(ch341_io, src, out, len);
}
int Arduino_verbose(void) { return ch341_io->verbose; }

void Arduino_rc522_init(void);
void Arduino_rc522_test(void);
void Arduino_rc522_wait(int timeout);
void Arduino_rc522_dump(void);
void Arduino_rc522_dump2(const uint8_t *key);

static void rc522_init(usbio_t *io) { (void)io; Arduino_rc522_init(); }
static void rc522_test(usbio_t *io) { (void)io; Arduino_rc522_test(); }
#endif
