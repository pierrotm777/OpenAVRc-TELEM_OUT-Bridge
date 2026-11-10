/*
 * OpenAVRc SX1276 SPI driver - FrSky R9 (LoRa 868/915 MHz).
 * Based on Multiprotocol SX1276_SPI.ino, GPLv3 or later.
 * SPI transactions reuse OpenAVRc RF_SPI_xfer and the existing RF CS pin.
 * Default: A7105 slot used exclusively for SX1276 (A7105=NO).
 * To use a separate CS: define RF_CS_SX1276_ACTIVE() and
 * RF_CS_SX1276_INACTIVE() in the target board header.
 */
#include "../OpenAVRc.h"

#if !defined(RF_CS_SX1276_ACTIVE) || !defined(RF_CS_SX1276_INACTIVE)
  #if defined(PROTO_HAS_A7105)
    #error "SX1276 default CS shares A7105. Use A7105=NO or assign separate RF_CS_SX1276_* macros."
  #endif
  #define RF_CS_SX1276_ACTIVE()    RF_CS_A7105_ACTIVE()
  #define RF_CS_SX1276_INACTIVE()  RF_CS_A7105_INACTIVE()
#endif

static void SX1276_WriteReg(uint8_t address, uint8_t data)
{
  RF_CS_SX1276_ACTIVE();
  RF_SPI_xfer(address | 0x80);
  RF_SPI_xfer(data);
  RF_CS_SX1276_INACTIVE();
}

static uint8_t SX1276_ReadReg(uint8_t address)
{
  RF_CS_SX1276_ACTIVE();
  RF_SPI_xfer(address & 0x7F);
  uint8_t result = RF_SPI_xfer(0);
  RF_CS_SX1276_INACTIVE();
  return result;
}

static void SX1276_WriteRegisterMulti(uint8_t address, const uint8_t *data, uint8_t length)
{
  RF_CS_SX1276_ACTIVE();
  RF_SPI_xfer(address | 0x80);
  while (length--) RF_SPI_xfer(*data++);
  RF_CS_SX1276_INACTIVE();
}

static void SX1276_ReadRegisterMulti(uint8_t address, uint8_t *data, uint8_t length)
{
  RF_CS_SX1276_ACTIVE();
  RF_SPI_xfer(address & 0x7F);
  while (length--) *data++ = RF_SPI_xfer(0);
  RF_CS_SX1276_INACTIVE();
}

static bool SX1276_DetectChip()
{
  return SX1276_ReadReg(SX1276_42_VERSION) == 0x12;
}

static void SX1276_SetTxRxMode(enum TXRX_State mode)
{
  // Optional RF switch controls for modules needing external TXEN/RXEN.
  // Define these four macros in board-specific header if required.
#if defined(RF_SX1276_TX_ON) && defined(RF_SX1276_TX_OFF) && defined(RF_SX1276_RX_ON) && defined(RF_SX1276_RX_OFF)
  if (mode == TX_EN) { RF_SX1276_RX_OFF(); RF_SX1276_TX_ON(); }
  else if (mode == RX_EN) { RF_SX1276_TX_OFF(); RF_SX1276_RX_ON(); }
  else { RF_SX1276_RX_OFF(); RF_SX1276_TX_OFF(); }
#else
  (void)mode;
#endif
}

static void SX1276_SetMode(bool lora, bool low_freq_mode, uint8_t mode)
{
  SX1276_WriteReg(SX1276_01_OPMODE,
     (lora ? 0x80 : 0x40) | (low_freq_mode ? 0x08 : 0) | (mode & 0x07));
}

static void SX1276_SetDetectOptimize(bool auto_if, uint8_t value)
{
  uint8_t val = SX1276_ReadReg(SX1276_31_DETECTOPTIMIZE);
  SX1276_WriteReg(SX1276_31_DETECTOPTIMIZE,
                  (val & 0x78) | (auto_if ? 0x80 : 0) | (value & 0x07));
}

static void SX1276_ConfigModem1(uint8_t bw, uint8_t cr, bool implicit)
{
  SX1276_WriteReg(SX1276_1D_MODEMCONFIG1, (bw << 4) | (cr << 1) | (implicit ? 1 : 0));
  if (bw == SX1276_MODEM_CONFIG1_BW_500KHZ) {
    SX1276_WriteReg(SX1276_36_LORA_REGHIGHBWOPTIMIZE1, 0x02);
    SX1276_WriteReg(SX1276_3A_LORA_REGHIGHBWOPTIMIZE2, 0x64);
  }
  else SX1276_WriteReg(SX1276_36_LORA_REGHIGHBWOPTIMIZE1, 0x03);
}

static void SX1276_ConfigModem2(uint8_t sf, bool continuous, bool crc)
{
  uint8_t previous = SX1276_ReadReg(SX1276_1E_MODEMCONFIG2);
  SX1276_WriteReg(SX1276_1E_MODEMCONFIG2,
                  (previous & 3) | (sf << 4) | (continuous ? 8 : 0) | (crc ? 4 : 0));
}

static void SX1276_ConfigModem3(bool low_data_rate, bool agc)
{
  uint8_t previous = SX1276_ReadReg(SX1276_26_MODEMCONFIG3);
  SX1276_WriteReg(SX1276_26_MODEMCONFIG3,
                  (previous & 3) | (low_data_rate ? 8 : 0) | (agc ? 4 : 0));
}

static void SX1276_SetPreambleLength(uint16_t count)
{
  uint8_t data[2] = {uint8_t(count >> 8), uint8_t(count)};
  SX1276_WriteRegisterMulti(SX1276_20_PREAMBLEMSB, data, 2);
}

static void SX1276_SetLna(uint8_t gain, bool boost)
{
  uint8_t previous = SX1276_ReadReg(SX1276_0C_LNA);
  SX1276_WriteReg(SX1276_0C_LNA, (previous & 0x04) | (gain << 5) | (boost ? 0x03 : 0));
}

static void SX1276_SetPaDac(bool high_power)
{
  uint8_t val = SX1276_ReadReg(SX1276_4D_PADAC);
  SX1276_WriteReg(SX1276_4D_PADAC, (val & 0xF8) | (high_power ? 7 : 4));
}

static void SX1276_SetPaConfig(uint8_t setting)
{
  SX1276_WriteReg(SX1276_09_PACONFIG, 0xF0 | (setting & 0x0F));
}

static void SX1276_WritePayloadToFifo(const uint8_t *payload, uint8_t length)
{
  SX1276_WriteReg(SX1276_22_PAYLOAD_LENGTH, length);
  SX1276_WriteReg(SX1276_0E_FIFOTXBASEADDR, 0);
  SX1276_WriteReg(SX1276_0D_FIFOADDRPTR, 0);
  SX1276_WriteRegisterMulti(SX1276_00_FIFO, payload, length);
}
