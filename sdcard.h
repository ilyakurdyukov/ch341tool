
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

static void sd_init(usbio_t *io) {
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
}

static void sd_info(usbio_t *io) {
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
			else {
				s = tmp > 0x8400 ? "4.XX and " : "";
				DBG_LOG("%u.XX", 4 + sd_specx);
			}
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
		c_size_mult = ((buf[9] << 8 | buf[10]) >> 7 & 7) + 2;
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
}

