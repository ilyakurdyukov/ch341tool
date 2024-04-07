#pragma once
#include <MFRC522Driver.h>

class MFRC522DriverCH341 : public MFRC522Driver {
public:
  bool init() override;
  void PCD_WriteRegister(const PCD_Register reg, const byte value) override;
  void PCD_WriteRegister(const PCD_Register reg, const byte count, byte *const values) override;
  byte PCD_ReadRegister(const PCD_Register reg) override;
  void PCD_ReadRegister(const PCD_Register reg, const byte count, byte *const values, const byte rxAlign = 0) override;
};

