/*
 ***************************************************************************
 * OpenAVRc - FrSky R9 SX1276 (FLEX 915 / FLEX 868 / FCC)
 * Based on Multiprotocol FrSkyR9_sx1276.ino / SX1276_SPI.ino (GPLv3+)
 * and OpenAVRc FRSKYX_cc2500.c (GPLv2 or later).
 * Reuses the OpenAVRc shared 144-byte pulses2MHz memory: no R9 global RAM.
 * EU (non FLEX) is deliberately excluded: EU packet builder is incomplete
 * in the provided reference code.
 ***************************************************************************
 */
#include "../OpenAVRc.h"

// Diagnostic: set to 1 ONLY to test that protocol menu selection stays stable.
// When set to 1, there is NO SX1276 access, NO RF transmission, and NO callback.
// Set back to 0 for normal use.
#define R9_DIAG_DISABLE_RADIO 1

// Reusable memory: indexes BYTE_P2M(1..9) are within the shared buffer.
#define R9_PHASE_P2M         BYTE_P2M(1)
#define R9_CHAN_SKIP_P2M     BYTE_P2M(2)
#define R9_HOP_P2M           BYTE_P2M(3)
#define R9_CH_OFF_P2M        BYTE_P2M(4)
#define R9_BIND_P2M          BYTE_P2M(5)
#define R9_REC_SEQ_P2M       BYTE_P2M(6)
#define R9_SEND_SEQ_P2M      BYTE_P2M(7)
#define R9_POWER_MEM_P2M     BYTE_P2M(8)

#define R9_FLEX915  (g_model.rfSubType == 0)
#define R9_FLEX868  (g_model.rfSubType == 1)
#define R9_FCC      (g_model.rfSubType == 2)
#define R9_TELEM    (g_model.rfOptionBool1)
#define R9_16CH     (g_model.rfOptionBool2)

#define R9_FLEX_FREQ 29
#define R9_FCC_FREQ  43

const static RfOptionSettingsvar_t RfOpt_FrskyR9_Ser[] PROGMEM = {
  /*rfProtoNeed*/PROTO_NEED_SPI | BOOL1USED, // TEST: hide both boolean options
  /*rfSubTypeMax*/2,
  /*rfOptionValue1Min*/-128,
  /*rfOptionValue1Max*/127,
  /*rfOptionValue2Min*/0,
  /*rfOptionValue2Max*/0,
  /*rfOptionValue3Max*/7,
};

const pm_char STR_SUBTYPE_FRSKYR9[] PROGMEM = "F915" "F868" "FCC ";
const pm_char STR_R9_16CH[] PROGMEM = INDENT "X16";

enum {
  FRSKYR9_FREQ,
  FRSKYR9_DATA,
  FRSKYR9_RX1,
  FRSKYR9_RX2,
};

// R9 packets use the same 16-entry lookup CRC variant as supplied MPM.
static uint16_t R9_frsky_crc(uint8_t *data, uint8_t len, uint16_t init=0)
{
  static const uint16_t table[] PROGMEM = {
    0x0000,0x1189,0x2312,0x329B,0x4624,0x57AD,0x6536,0x74BF,
    0x8C48,0x9DC1,0xAF5A,0xBED3,0xCA6C,0xDBE5,0xE97E,0xF8F7
  };
  uint16_t crc = init;
  for(uint8_t i=0; i<len; ++i) {
    uint8_t x = (uint8_t)(crc >> 8) ^ data[i];
    uint16_t word = pgm_read_word(&table[x & 0x0F]);
    crc = (uint16_t)(crc << 8) ^ (word ^ (uint16_t)(0x1081u * (x >> 4)));
  }
  return crc;
}

static void FrSkyR9_set_frequency()
{
  uint16_t num;
  R9_HOP_P2M += R9_CHAN_SKIP_P2M;
  if (R9_FCC) {
    R9_HOP_P2M %= R9_FCC_FREQ;
    num = (uint16_t)R9_HOP_P2M * 32u + 0xE200;
  }
  else {
    // R9 FLEX 868 binds at FLEX 915, like MPM/Deviation.
    R9_HOP_P2M %= R9_FLEX_FREQ;
    num = R9_HOP_P2M;
    if (R9_HOP_P2M >= R9_FLEX_FREQ - 2)
      num += (int8_t)R9_CHAN_SKIP_P2M - R9_FLEX_FREQ + 2;
    num = (num << 5) + ((R9_FLEX868 && !R9_BIND_P2M) ? 0xD700 : 0xE4C0);
  }
  // SX1276 synthesizer step ~61 Hz per LSB: FREQFINE adjustable.
  uint32_t frf = ((uint32_t)num << 8) + (int8_t)g_model.rfOptionValue1;
  uint8_t data[3] = {uint8_t(frf >> 16), uint8_t(frf >> 8), uint8_t(frf)};
  SX1276_WriteRegisterMulti(SX1276_06_FRFMSB, data, 3);
}

static uint16_t FrSkyR9_scaleForPXX(uint8_t channel)
{
  int16_t value = ((FULL_CHANNEL_OUTPUTS(channel)) * 3 / 4) + 1024;
  value = limit<int16_t>(1, value, 2046);
  if(channel >= 8) value |= 2048;
  return (uint16_t)value;
}

static void FrSkyR9_channels()
{
  uint8_t ch = R9_CH_OFF_P2M;
  packet_p2M[6] = 0;
  packet_p2M[7] = 0;
  for(uint8_t i=8; i<20; i+=3) {
    uint16_t c1=FrSkyR9_scaleForPXX(ch++);
    uint16_t c2=FrSkyR9_scaleForPXX(ch++);
    packet_p2M[i]   = uint8_t(c1);
    packet_p2M[i+1] = uint8_t((c1 >> 8) | (c2 << 4));
    packet_p2M[i+2] = uint8_t(c2 >> 4);
  }
  R9_CH_OFF_P2M = R9_16CH ? (R9_CH_OFF_P2M ^ 8) : 0;
}

static void FrSkyR9_build_packet()
{
#if defined(X_ANY)
  Xany_scheduleTx_AllInstance();
#endif
  packet_p2M[0] = temp_rfid_addr_p2M[1];
  packet_p2M[1] = temp_rfid_addr_p2M[2];
  packet_p2M[2] = temp_rfid_addr_p2M[3];
  packet_p2M[3] = R9_HOP_P2M;
  packet_p2M[4] = R9_CHAN_SKIP_P2M;
  packet_p2M[5] = RXNUM;
  FrSkyR9_channels();
  if(R9_BIND_P2M) {
    packet_p2M[6] = 0x01;
    if(R9_FLEX868) packet_p2M[6] |= 0x20;
    if(!R9_TELEM) packet_p2M[6] |= 0x40;
    if(R9_16CH) packet_p2M[6] |= 0x80;
  }
  // Uplink S.Port sending is not implemented (0 bytes). Sequence handshake
  // and S.Port reception below are implemented independently.
  packet_p2M[20] = (R9_REC_SEQ_P2M << 4) | R9_SEND_SEQ_P2M;
  packet_p2M[21] = 0;
  packet_p2M[22] = 0;
  packet_p2M[23] = 0;
  uint16_t crc = R9_frsky_crc(packet_p2M, 24);
  packet_p2M[24] = uint8_t(crc);       // R9 CRC order: low byte first
  packet_p2M[25] = uint8_t(crc >> 8);
}

#if defined(FRSKY)
static void FrSkyR9_check_telemetry(uint8_t *pkt)
{
  // R9 RX frame: 0=RSSI/voltage, 1=sequence, 2=count,
  // 3..8=up to six S.Port bytes, 9..10=TX ID, 11..12=CRC.
  if(pkt[9] != temp_rfid_addr_p2M[1] || pkt[10] != temp_rfid_addr_p2M[2]) return;
  uint16_t expected = (uint16_t)pkt[11] | ((uint16_t)pkt[12] << 8);
  uint16_t init = (uint16_t)temp_rfid_addr_p2M[1] | ((uint16_t)temp_rfid_addr_p2M[2] << 8);
  if(R9_frsky_crc(pkt, 11, init) != expected) return;

  frskyStreaming = frskyStreaming ? FRSKY_TIMEOUT10ms : FRSKY_TIMEOUT_FIRST;
  if(pkt[0] & 0x80) telemetryData.rssi[0].set(pkt[0] & 0x7F);
  // No second RSSI/LQI byte in the R9 13-byte telemetry packet.
  if(((pkt[1] & 0x0F) == 8) || ((pkt[1] >> 4) == 8)) {
    R9_REC_SEQ_P2M=8;
    R9_SEND_SEQ_P2M=0;
    parseTelemFrskyByte(START_STOP);
  }
  else if((pkt[1] & 3) == (R9_REC_SEQ_P2M & 3)) {
    R9_REC_SEQ_P2M=(R9_REC_SEQ_P2M+1)&3;
    if(pkt[2] && pkt[2]<=6)
      for(uint8_t i=0;i<pkt[2];i++) parseTelemFrskyByte(pkt[3+i]);
  }
  else R9_REC_SEQ_P2M = (R9_REC_SEQ_P2M & 3) | 4; // ask resend
}
#endif

static void FrSkyR9_init()
{
  // Generate persistent MPM-compatible transmitter ID (not model number only).
  loadrfidaddr();
  R9_CHAN_SKIP_P2M = 1 + (uint8_t)(((uint16_t)temp_rfid_addr_p2M[0] +
                                     temp_rfid_addr_p2M[1] + temp_rfid_addr_p2M[2] +
                                     temp_rfid_addr_p2M[3]) % 24);
  R9_HOP_P2M=0;
  R9_CH_OFF_P2M=0;
  R9_REC_SEQ_P2M=0;
  R9_SEND_SEQ_P2M=8;
  R9_POWER_MEM_P2M=0xFF;
  R9_PHASE_P2M=FRSKYR9_FREQ;

  SX1276_SetMode(true,false,SX1276_OPMODE_SLEEP);
  SX1276_SetMode(true,false,SX1276_OPMODE_STDBY);
  SX1276_SetDetectOptimize(true, SX1276_DETECT_OPTIMIZE_SF6);
  SX1276_ConfigModem1(SX1276_MODEM_CONFIG1_BW_500KHZ,
                      SX1276_MODEM_CONFIG1_CODING_RATE_4_5,true);
  SX1276_ConfigModem2(6,false,false);
  SX1276_ConfigModem3(false,false);
  SX1276_SetPreambleLength(9);
  SX1276_WriteReg(SX1276_37_DETECTIONTHRESHOLD,SX1276_MODEM_DETECTION_THRESHOLD_SF6);
  SX1276_SetLna(1,true);
  SX1276_WriteReg(SX1276_24_HOPPERIOD,0);
  SX1276_SetPaDac(false);
  SX1276_WriteReg(SX1276_0B_OCP,0x20 | 27);
  SX1276_WriteReg(SX1276_11_IRQFLAGSMASK,0);
  SX1276_WriteReg(SX1276_0F_FIFORXBASEADDR,0);
  SX1276_SetTxRxMode(TX_EN);
}

static uint16_t FrSkyR9_callback()
{
  switch(R9_PHASE_P2M) {
  case FRSKYR9_FREQ: {
    SCHEDULE_MIXER_END_IN_US(20000);
    SX1276_SetMode(true,false,SX1276_OPMODE_STDBY);
    FrSkyR9_set_frequency();
    // Follow the OpenAVRc RF power option and range-check mode.
    uint8_t r9power = systemBolls.rangeModeIsOn ? TXPOWER_1 : g_model.rfOptionValue3;
    if(R9_POWER_MEM_P2M != r9power) {
      SX1276_SetPaConfig((r9power <= 7) ? r9power * 2 : 14);
      R9_POWER_MEM_P2M = r9power;
    }
    FrSkyR9_build_packet();
    R9_PHASE_P2M=FRSKYR9_DATA;
    return 460;
  }

  case FRSKYR9_DATA:
    SX1276_SetTxRxMode(TX_EN);
    SX1276_WritePayloadToFifo(packet_p2M,26);
    SX1276_SetMode(true,false,SX1276_OPMODE_TX);
#if defined(FRSKY)
    if(R9_TELEM && !R9_BIND_P2M) {
      R9_PHASE_P2M=FRSKYR9_RX1;
      return 11140;
    }
#endif
    R9_PHASE_P2M=FRSKYR9_FREQ;
    return 19540; // 460 + 19540 = 20 ms

  case FRSKYR9_RX1:
    SX1276_SetMode(true,false,SX1276_OPMODE_STDBY);
    SX1276_WriteReg(SX1276_22_PAYLOAD_LENGTH,13);
    SX1276_WriteReg(SX1276_0D_FIFOADDRPTR,0);
    SX1276_SetTxRxMode(RX_EN);
    SX1276_WriteReg(SX1276_12_REGIRQFLAGS,0xFF);
    SX1276_SetMode(true,false,SX1276_OPMODE_RXCONTINUOUS);
    R9_PHASE_P2M=FRSKYR9_RX2;
    return 7400;

  case FRSKYR9_RX2:
#if defined(FRSKY)
    if((SX1276_ReadReg(SX1276_12_REGIRQFLAGS) & 0x70) == 0x50 &&
       SX1276_ReadReg(SX1276_13_REGRXNBBYTES)==13) {
      SX1276_WriteReg(SX1276_0D_FIFOADDRPTR,
                      SX1276_ReadReg(SX1276_10_FIFORXCURRENTADDR));
      SX1276_ReadRegisterMulti(SX1276_00_FIFO, packet_p2M,13);
      FrSkyR9_check_telemetry(packet_p2M);
    }
#endif
    R9_PHASE_P2M=FRSKYR9_FREQ;
    return 1000;
  }
  R9_PHASE_P2M=FRSKYR9_FREQ;
  return 20000;
}

static uint16_t FRSKYR9_bind_cb()
{
  uint16_t time=FrSkyR9_callback();
  heartbeat |= HEART_TIMER_PULSES;
  CALCULATE_LAT_JIT();
  return time*2;
}

static uint16_t FRSKYR9_cb()
{
  uint16_t time=FrSkyR9_callback();
  heartbeat |= HEART_TIMER_PULSES;
  CALCULATE_LAT_JIT();
  return time*2;
}

static void FRSKYR9_initialize(uint8_t bind)
{
#if R9_DIAG_DISABLE_RADIO
  // Menu-only test. This deliberately does not access the SPI bus.
  (void)bind;
  return;
#else
  R9_BIND_P2M=bind;
  if(!SX1276_DetectChip()) {
    // Hardware/CS wiring to be checked; avoid a ghost RF transmitter.
    return;
  }
  FrSkyR9_init();
  PROTO_Start_Callback(bind ? FRSKYR9_bind_cb : FRSKYR9_cb);
#endif
}

const void *FRSKYR9_Cmds(enum ProtoCmds cmd)
{
  switch(cmd) {
    case PROTOCMD_INIT:
      FRSKYR9_initialize(0);
      return 0;
    case PROTOCMD_RESET:
      PROTO_Stop_Callback();
#if !R9_DIAG_DISABLE_RADIO
      SX1276_SetMode(true,false,SX1276_OPMODE_SLEEP);
      SX1276_SetTxRxMode(TXRX_OFF);
#endif
      return 0;
    case PROTOCMD_BIND:
      FRSKYR9_initialize(1);
      return 0;
    case PROTOCMD_GETOPTIONS:
      SetRfOptionSettings(pgm_get_far_address(RfOpt_FrskyR9_Ser),
                          STR_SUBTYPE_FRSKYR9,
                          STR_RFTUNEFINE,
						  STR_DUMMY,
						  STR_RFPOWER,
                          STR_TELEMETRY,
						  STR_DUMMY,
						  STR_DUMMY); // TEST: no TELEMETRY/X16
      return 0;
    default: return 0;
  }
}
