#include "MFRC522DriverCH341.h"

#define ERR_EXIT(...) \
	do { fprintf(stderr, __VA_ARGS__); exit(1); } while (0)

extern "C"
void Arduino_spi_stream(const uint8_t *src, uint8_t *out, unsigned len);

bool MFRC522DriverCH341::init() {
	return true;
}

void MFRC522DriverCH341::PCD_WriteRegister(const PCD_Register reg, const byte val) {
	uint8_t buf[2] = { (uint8_t)(reg << 1), val };
	Arduino_spi_stream(buf, NULL, 2);
}

void MFRC522DriverCH341::PCD_WriteRegister(const PCD_Register reg, const byte len, byte *const src) {
	uint8_t buf[1 + 25];
	if (!len || len >= sizeof(buf))
		ERR_EXIT("write_buf: unexpected length\n");
	buf[0] = reg << 1;
	memcpy(buf + 1, src, len);
	Arduino_spi_stream(buf, NULL, len + 1);
}

byte MFRC522DriverCH341::PCD_ReadRegister(const PCD_Register reg) {
	uint8_t buf[2] = { (uint8_t)(0x80 | reg << 1), 0 };
	Arduino_spi_stream(buf, buf, 2);
	return buf[1];
}

void MFRC522DriverCH341::PCD_ReadRegister(const PCD_Register reg,
		const byte len, byte *const dst, const byte rxAlign) {
	uint8_t buf[1 + 64];
	int mask = 0xff << rxAlign;
	if (!len) return;
	if (len >= sizeof(buf))
		ERR_EXIT("read_buf: unexpected length\n");
	memset(buf, 0x80 | reg << 1, len);
	buf[len] = 0;
	Arduino_spi_stream(buf, buf, len + 1);
	dst[0] = (dst[0] & ~mask) | (buf[1] & mask);
	if (len > 1) memcpy(dst + 1, buf + 2, len - 1);
}

#include "MFRC522Debug.h"

static MFRC522DriverCH341 driver;
static MFRC522 mfrc522 { driver };
static Print Serial { stderr };

extern "C" {

void Arduino_rc522_init() {
  mfrc522.PCD_Init();
  MFRC522Debug::PCD_DumpVersionToSerial(mfrc522, Serial);
}

void Arduino_rc522_test() {
	bool result = mfrc522.PCD_PerformSelfTest(); // Perform the test.
	Serial.println(F("-----------------------------"));
	Serial.print(F("Result: "));
	if (result)
		Serial.println(F("OK"));
	else
		Serial.println(F("DEFECT or UNKNOWN"));
	Serial.println();
}

void Arduino_rc522_wait(int timeout) {
#if 0
  unsigned start = millis();
  while (millis() - start < timeout * 1000u) {
#else
	int i;
	for (i = 0; i < timeout * 100; i++) {
#endif
		if (mfrc522.PICC_IsNewCardPresent()) return;
		delay(1);
	}
	ERR_EXIT("rc522_wait: timeout reached\n");
}

using PICC_Type = MFRC522Constants::PICC_Type;
using Uid = MFRC522Constants::Uid;
using MIFARE_Key = MFRC522Constants::MIFARE_Key;

void Arduino_rc522_dump(void) {
	if (!mfrc522.PICC_ReadCardSerial())
		ERR_EXIT("rc522_info: failed to read serial\n");
	MFRC522Debug::PICC_DumpToSerial(mfrc522, Serial, &mfrc522.uid);
}

void Arduino_rc522_dump2(const uint8_t *key) {
	if (!mfrc522.PICC_ReadCardSerial())
		ERR_EXIT("rc522_info: failed to read serial\n");

	Uid *uid = &mfrc522.uid;
  MFRC522Debug::PICC_DumpDetailsToSerial(mfrc522, Serial, uid);
  PICC_Type piccType = mfrc522.PICC_GetType(uid->sak);
	MFRC522Debug::PICC_DumpMifareClassicToSerial(mfrc522, Serial, uid, piccType, (MIFARE_Key*)key);
}

}
