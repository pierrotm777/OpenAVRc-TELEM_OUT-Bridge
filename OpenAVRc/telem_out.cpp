/*
 * OpenAVRc - Serial1 telemetry output
 *
 * Dedicated TELEM_OUT layer.  Bluetooth/HC05 code remains untouched.
 */
#include "OpenAVRc.h"
#include "Serial1.h"
#include "telem_out.h"

#define TELEM_OUT_CRSF_SYNC  0xC8 // Same CRSF sync/address used by the validated BLE simulator
#define TELEM_OUT_CRSF_TYPE_GPS          0x02
#define TELEM_OUT_CRSF_TYPE_VARIO        0x07
#define TELEM_OUT_CRSF_TYPE_BATTERY      0x08
#define TELEM_OUT_CRSF_TYPE_BARO_ALT     0x09
#define TELEM_OUT_CRSF_TYPE_RPM          0x0C
#define TELEM_OUT_CRSF_TYPE_TEMPERATURE  0x0D
#define TELEM_OUT_CRSF_TYPE_VOLTAGES     0x0E
#define TELEM_OUT_CRSF_TYPE_LINK         0x14
#define TELEM_OUT_CRSF_TYPE_CHANNELS     0x16


#if (SERIAL_PROTOCOL == CRSF)
// Local snapshot of the last complete CRSF Link Statistics frame seen by
// CRSF_SERIAL.c. No modification of CRSF_SERIAL.c is required.
static uint8_t telemOutCrsfLinkStats[10] = {0};
static uint8_t telemOutCrsfLinkValid = 0;

static void telemOutCaptureCrsfLink()
{
  if (!IS_CRSF_PROTOCOL(s_current_protocol))
    return;

  uint8_t frame[14];
  uint8_t size;
  uint8_t count;

  // CRSF_SERIAL.c is included through OpenAVRc.h in this build, therefore its
  // receive buffer is visible here. Take an atomic snapshot so the UART ISR
  // cannot modify the bytes while they are copied.
#if defined(__AVR__)
  const uint8_t oldSreg = SREG;
  cli();
#endif

  size = crsf_menu_rx_size;
  count = crsf_menu_rx_count;

  if (count == 0U && size == 14U && crsf_menu_rxbuf[2] == CRSF_TYPE_LINK)
    {
      for (uint8_t i=0; i<14U; ++i)
        frame[i] = crsf_menu_rxbuf[i];
    }
  else
    {
#if defined(__AVR__)
      SREG = oldSreg;
#endif
      return;
    }

#if defined(__AVR__)
  SREG = oldSreg;
#endif

  // Validate the complete frame again before publishing the local copy.
  if (CRSF_MenuCRC(&frame[2], 11U) != frame[13])
    return;

  for (uint8_t i=0; i<10U; ++i)
    telemOutCrsfLinkStats[i] = frame[3U+i];

  telemOutCrsfLinkValid = 1U;
}
#endif

static uint8_t telemOutCrsfCrc8(uint8_t crc, uint8_t data)
{
  crc ^= data;
  for (uint8_t i = 0; i < 8; ++i)
    {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0xD5);
      else
        crc <<= 1;
    }
  return crc;
}

static uint8_t telemOutCrsfCrc(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
    crc = telemOutCrsfCrc8(crc, *data++);
  return crc;
}

static void telemOutPutU16BE(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value >> 8);
  dst[1] = (uint8_t)value;
}

static void telemOutPutI16BE(uint8_t *dst, int16_t value)
{
  telemOutPutU16BE(dst, (uint16_t)value);
}

static void telemOutPutU24BE(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 16);
  dst[1] = (uint8_t)(value >> 8);
  dst[2] = (uint8_t)value;
}

static void telemOutPutI24BE(uint8_t *dst, int32_t value)
{
  const uint32_t raw = (uint32_t)value;
  dst[0] = (uint8_t)(raw >> 16);
  dst[1] = (uint8_t)(raw >> 8);
  dst[2] = (uint8_t)raw;
}

static void telemOutPutI32BE(uint8_t *dst, int32_t value)
{
  const uint32_t raw = (uint32_t)value;
  dst[0] = (uint8_t)(raw >> 24);
  dst[1] = (uint8_t)(raw >> 16);
  dst[2] = (uint8_t)(raw >> 8);
  dst[3] = (uint8_t)raw;
}

#if defined(FRSKY) && defined(GPS)
static int32_t telemOutGpsCoord1e7(uint16_t bp, uint16_t ap, uint8_t hemisphere)
{
  // OpenAVRc stores GPS coordinates as DDMM.mmmm:
  //   bp = DDMM
  //   ap = mmmm
  // Convert to signed decimal degrees x 1e7 without floating point.
  const uint16_t degrees = bp / 100U;
  const uint16_t minutes = bp % 100U;
  const uint32_t minutes10000 = (uint32_t)minutes * 10000UL + ap;

  // 1 unit of 0.0001 minute = 16.666666... units of degree x 1e7.
  const uint32_t decimalPart = (minutes10000 * 50UL + 1UL) / 3UL;

  int32_t result = (int32_t)((uint32_t)degrees * 10000000UL + decimalPart);

  if (hemisphere == 'S' || hemisphere == 'W')
    result = -result;

  return result;
}
#endif

static void telemOutSendBattery()
{
  // CRSF Battery Sensor frame 0x08:
  // voltage 0.1V, current 0.1A, capacity mAh, remaining %.
  uint8_t frame[12];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 10;
  frame[2] = TELEM_OUT_CRSF_TYPE_BATTERY;

#if defined(FRSKY)
  // OpenAVRc common telemetry -> standard CRSF Battery Sensor:
  // VFAS               : 0.1 V
  // Current            : 0.1 A
  // CurrentConsumption : mAh
  // FuelLevel          : remaining %, when a real percentage is available
  const uint16_t voltage = telemetryData.value.vfas;
  const uint16_t current = telemetryData.value.current;
  const uint32_t capacity = telemetryData.value.currentConsumption;
  const uint8_t remaining =
      (telemetryData.value.fuelLevel <= 100U) ?
      (uint8_t)telemetryData.value.fuelLevel : 0U;
#else
  const uint16_t voltage = 0;
  const uint16_t current = 0;
  const uint32_t capacity = 0;
  const uint8_t remaining = 0;
#endif

  telemOutPutU16BE(&frame[3], voltage);
  telemOutPutU16BE(&frame[5], current);
  telemOutPutU24BE(&frame[7], capacity);

  frame[10] = remaining;
  frame[11] = telemOutCrsfCrc(&frame[2], 9);

  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendVario()
{
  // CRSF Vario frame 0x07:
  // signed vertical speed in cm/s.
  uint8_t frame[6];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 4;
  frame[2] = TELEM_OUT_CRSF_TYPE_VARIO;

#if defined(FRSKY)
  const int16_t vario = telemetryData.value.varioSpeed;
#else
  const int16_t vario = 0;
#endif

  telemOutPutI16BE(&frame[3], vario);
  frame[5] = telemOutCrsfCrc(&frame[2], 3);

  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendGps()
{
  // CRSF GPS frame 0x02, 15-byte payload:
  // latitude int32 deg*1e7
  // longitude int32 deg*1e7
  // ground speed uint16 km/h*10
  // heading uint16 deg*100
  // altitude uint16 metres + 1000
  // satellites uint8
  uint8_t frame[19];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 17;
  frame[2] = TELEM_OUT_CRSF_TYPE_GPS;

  int32_t latitude = 0;
  int32_t longitude = 0;
  uint16_t speed = 0;
  uint16_t heading = 0;
  uint16_t altitude = 1000;
  uint8_t satellites = 0;

#if defined(FRSKY) && defined(GPS)
  if (telemetryData.value.gpsFix)
    {
      latitude = telemOutGpsCoord1e7(telemetryData.value.gpsLatitude_bp,
                                     telemetryData.value.gpsLatitude_ap,
                                     telemetryData.value.gpsLatitudeNS);

      longitude = telemOutGpsCoord1e7(telemetryData.value.gpsLongitude_bp,
                                      telemetryData.value.gpsLongitude_ap,
                                      telemetryData.value.gpsLongitudeEW);

      // OpenAVRc internal GPS speed is 0.1 knot.
      // CRSF GPS speed is 0.1 km/h.
      speed = (uint16_t)(((uint32_t)telemetryData.value.gpsSpeed_bp * 1852UL + 500UL) / 1000UL);

      heading = (uint16_t)((uint32_t)telemetryData.value.gpsCourse_bp * 100UL +
                           telemetryData.value.gpsCourse_ap);

      int32_t alt = (int32_t)telemetryData.value.gpsAltitude + 1000L;
      if (alt < 0)
        alt = 0;
      else if (alt > 65535L)
        alt = 65535L;
      altitude = (uint16_t)alt;

      // Temporary test value until NUMSAT is available from the common
      // OpenAVRc/S.Port telemetry path.
      satellites = 10;
    }
#endif

  telemOutPutI32BE(&frame[3], latitude);
  telemOutPutI32BE(&frame[7], longitude);
  telemOutPutU16BE(&frame[11], speed);
  telemOutPutU16BE(&frame[13], heading);
  telemOutPutU16BE(&frame[15], altitude);
  frame[17] = satellites;

  frame[18] = telemOutCrsfCrc(&frame[2], 16);

  telemOutSend(frame, sizeof(frame));
}



static uint8_t telemOutPowerMwToCrsf(uint16_t mw)
{
  // CRSF 0x14 TX-power enum:
  // 0=0mW, 1=10mW, 2=25mW, 3=100mW, 4=500mW,
  // 5=1000mW, 6=2000mW, 7=250mW, 8=50mW.
  if (mw >= 1500U) return 6;
  if (mw >= 750U)  return 5;
  if (mw >= 375U)  return 4;
  if (mw >= 175U)  return 7;
  if (mw >= 75U)   return 3;
  if (mw >= 38U)   return 8;
  if (mw >= 18U)   return 2;
  if (mw >= 5U)    return 1;
  return 0;
}

static uint8_t telemOutGetTxPowerCrsf()
{
  // DSM serial: OpenAVRc does not know the RF power of the external module.
#if (SERIAL_PROTOCOL == DSM)
  if (IS_DSM2_SERIAL_PROTOCOL(s_current_protocol))
    return 0;
#endif

  // MULTI: use Option1 only when the Multi module declares it as RF POWER.
  // MPM OPTION_RFPOWER uses logarithmic steps:
  // 1.6,2,2.5,3.2,4,5,6.3,7.9,10,13,16,20,25,32,40,50,
  // then 63,79,100,125,160,200,250,320,400,500,630,800,1000...
#if (SERIAL_PROTOCOL == MULTIMODULE)
  if (IS_MULTIMODULE_PROTOCOL(s_current_protocol))
    {
      if (RfOptionSettings.rfOptionValue1Name == STR_RFPOWER)
        {
          const uint8_t p = (uint8_t)g_model.rfOptionValue1;
          if (p >= 28U) return 5; // >=1000mW
          if (p >= 25U) return 4; // 500..800mW
          if (p >= 22U) return 7; // 250..400mW
          if (p >= 18U) return 3; // 100..200mW
          if (p >= 15U) return 8; // 50..79mW
          if (p >= 12U) return 2; // 25..40mW
          if (p >= 8U)  return 1; // 10..20mW
        }
      return 0;
    }
#endif

  // CRSF/ELRS: OpenAVRc menu values are:
  // 0=--, 1=DYN, 2=10, 3=25, 4=50, 5=100,
  // 6=250, 7=500, 8=1000, 9=2000 mW.
#if (SERIAL_PROTOCOL == CRSF)
  if (IS_CRSF_PROTOCOL(s_current_protocol))
    {
      switch (g_model.rfOptionValue3)
        {
          case 2: return 1; // 10mW
          case 3: return 2; // 25mW
          case 4: return 8; // 50mW
          case 5: return 3; // 100mW
          case 6: return 7; // 250mW
          case 7: return 4; // 500mW
          case 8: return 5; // 1000mW
          case 9: return 6; // 2000mW
          default: return 0; // -- or DYN: actual instantaneous power unknown
        }
    }
#endif

  // Internal SPI protocols: RFPowerOut is the power value already computed
  // by the active RF-chip driver for the GUI. OpenAVRc stores it in 0.01mW.
#if defined(SPIMODULES)
  if (IS_SPIMODULES_PROTOCOL(s_current_protocol))
    {
      const uint16_t mw = (uint16_t)((RFPowerOut + 50U) / 100U);
      return telemOutPowerMwToCrsf(mw);
    }
#endif

  return 0;
}

static void telemOutSendLink()
{
  // CRSF Link Statistics 0x14, 10-byte payload:
  // uplink RSSI1, RSSI2, LQ, SNR, active antenna, RF mode, TX power,
  // downlink RSSI, LQ, SNR.
  //
  // OpenAVRc has two common link values in telemetryData.rssi[].
  // Preserve those values; fields with no common equivalent stay at zero.
  uint8_t frame[14];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 12;
  frame[2] = TELEM_OUT_CRSF_TYPE_LINK;

#if defined(FRSKY)
  uint8_t rssi = telemetryData.rssi[0].value;
  uint8_t quality = telemetryData.rssi[1].value;
#else
  uint8_t rssi = 0;
  uint8_t quality = 0;
#endif

  if (quality > 100U)
    quality = 100U;

#if (SERIAL_PROTOCOL == CRSF)
  if (IS_CRSF_PROTOCOL(s_current_protocol) && telemOutCrsfLinkValid)
    {
      // CRSF active: reuse the exact Link Statistics payload received from
      // the ELRS module: RSSI1/RSSI2, LQ/SNR, antenna, RF mode and PWR.
      for (uint8_t i=0; i<10U; ++i)
        frame[3U+i] = telemOutCrsfLinkStats[i];
    }
  else
#endif
    {
      // DSM / MULTI / SPI, or CRSF before the first valid 0x14:
      // unavailable information is deliberately transmitted as zero.
      frame[3]  = rssi;
      frame[4]  = rssi;
      frame[5]  = quality;
      frame[6]  = 0;
      frame[7]  = 0;
      frame[8]  = 0;
      frame[9]  = telemOutGetTxPowerCrsf();
      frame[10] = rssi;
      frame[11] = quality;
      frame[12] = 0;
    }

  frame[13] = telemOutCrsfCrc(&frame[2], 11);
  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendBaroAltitude()
{
  // CRSF Barometric Altitude 0x09:
  // bit15=0 => altitude in 0.1 m with +1000 m offset.
  uint8_t frame[6];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 4;
  frame[2] = TELEM_OUT_CRSF_TYPE_BARO_ALT;

#if defined(FRSKY)
  // OpenAVRc stores baroAltitude in metres.
  int32_t raw = (int32_t)telemetryData.value.baroAltitude * 10L + 10000L;
#else
  int32_t raw = 10000L;
#endif

  if (raw < 0L)
    raw = 0L;
  else if (raw > 0x7FFF)
    raw = 0x7FFF;

  telemOutPutU16BE(&frame[3], (uint16_t)raw);
  frame[5] = telemOutCrsfCrc(&frame[2], 3);
  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendRpm()
{
  // oXs/OpenAVRc compatible CRSF RPM 0x0C:
  // source_id 0 + signed 24-bit RPM.
  uint8_t frame[8];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 6;
  frame[2] = TELEM_OUT_CRSF_TYPE_RPM;
  frame[3] = 0; // source_id

#if defined(FRSKY)
  int32_t rpm = telemetryData.value.rpm;
#else
  int32_t rpm = 0;
#endif

  if (rpm < 0)
    rpm = 0;
  if (rpm > 0x7FFFFF)
    rpm = 0x7FFFFF;

  telemOutPutI24BE(&frame[4], rpm);
  frame[7] = telemOutCrsfCrc(&frame[2], 5);
  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendTemperature()
{
  // oXs/OpenAVRc compatible CRSF Temperature 0x0D:
  // source_id 0, T1 and T2, signed 0.1 deg C.
  uint8_t frame[9];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 7;
  frame[2] = TELEM_OUT_CRSF_TYPE_TEMPERATURE;
  frame[3] = 0; // source_id: T1 followed by T2

#if defined(FRSKY)
  int32_t t1 = (int32_t)telemetryData.value.temperature1 * 10L;
  int32_t t2 = (int32_t)telemetryData.value.temperature2 * 10L;
#else
  int32_t t1 = 0;
  int32_t t2 = 0;
#endif

  if (t1 < -32768L) t1 = -32768L;
  if (t1 >  32767L) t1 =  32767L;
  if (t2 < -32768L) t2 = -32768L;
  if (t2 >  32767L) t2 =  32767L;

  telemOutPutI16BE(&frame[4], (int16_t)t1);
  telemOutPutI16BE(&frame[6], (int16_t)t2);
  frame[8] = telemOutCrsfCrc(&frame[2], 6);
  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendCells()
{
  // CRSF Voltage Group 0x0E:
  // source_id 0 + six cell voltages in millivolts.
  uint8_t frame[17];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 15;
  frame[2] = TELEM_OUT_CRSF_TYPE_VOLTAGES;
  frame[3] = 0; // source_id 0 = cell group

  for (uint8_t i = 0; i < 6U; ++i)
    {
#if defined(FRSKY)
      // OpenAVRc cellVolts[] is in 0.02 V units.
      const uint16_t mv = (uint16_t)telemetryData.value.cellVolts[i] * 20U;
#else
      const uint16_t mv = 0;
#endif
      telemOutPutU16BE(&frame[4U + i * 2U], mv);
    }

  frame[16] = telemOutCrsfCrc(&frame[2], 14);
  telemOutSend(frame, sizeof(frame));
}

static void telemOutSendRxBatt()
{
  // CRSF Voltage Group 0x0E:
  // source_id 128 = receiver-local battery voltage, first value in millivolts.
  uint8_t frame[7];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 5;
  frame[2] = TELEM_OUT_CRSF_TYPE_VOLTAGES;
  frame[3] = 128;

#if defined(FRSKY)
  const uint16_t mv = (uint16_t)telemetryData.rxBatt * 100U;
#else
  const uint16_t mv = 0;
#endif

  telemOutPutU16BE(&frame[4], mv);
  frame[6] = telemOutCrsfCrc(&frame[2], 4);
  telemOutSend(frame, sizeof(frame));
}


static void telemOutSendRcChannels8()
{
  // Standard CRSF RC Channels Packed 0x16.
  // CH1..CH8 are the final OpenAVRc outputs after mixers.
  // CH9..CH16 stay at the CRSF midpoint so the Android viewer receives
  // a complete standards-compatible 16-channel frame.
  uint8_t frame[26];

  frame[0] = TELEM_OUT_CRSF_SYNC;
  frame[1] = 24; // type + 22-byte payload + CRC
  frame[2] = TELEM_OUT_CRSF_TYPE_CHANNELS;

  uint32_t bits = 0;
  uint8_t bitsAvailable = 0;
  uint8_t out = 3;

  for (uint8_t i = 0; i < 16U; ++i)
    {
      int16_t value;

      if (i < 8U)
        {
          // Same conversion used by OpenAVRc's native CRSF RF protocol.
          value = calcRESXto1000(FULL_CHANNEL_OUTPUTS(i)) * 8 / 10;
          value += 992;

          if (value < 172)
            value = 172;
          else if (value > 1811)
            value = 1811;
        }
      else
        {
          value = 992;
        }

      bits |= (uint32_t)(uint16_t)value << bitsAvailable;
      bitsAvailable += 11U;

      while (bitsAvailable >= 8U)
        {
          frame[out++] = (uint8_t)(bits & 0xFFU);
          bits >>= 8;
          bitsAvailable -= 8U;
        }
    }

  // 16 * 11 bits = exactly 22 bytes, so out must now be 25.
  frame[25] = telemOutCrsfCrc(&frame[2], 23);
  telemOutSend(frame, sizeof(frame));
}

void telemOutInit()
{
#if defined(CPUM2560) && !defined(SIMU)
  // The ESP32-C3 uses the same switched supply as the HC05 module.
  BT_POWER_ON();
#endif

  // Serial1, 115200 baud, 8N1.
  BT_Ser_Init(0);
}

void telemOutMain()
{
#if (SERIAL_PROTOCOL == CRSF)
  // Poll on every main-loop pass, not only when a telemetry frame is sent.
  // This maximizes the chance of catching the validated 0x14 frame before
  // another CRSF frame reuses the common RX buffer.
  telemOutCaptureCrsfLink();
#endif
  // One CRSF frame every 50 ms.
  // Even slots: CH1..CH8 packed in a standard 0x16 frame -> 10 Hz display.
  // Odd slots: one telemetry group from the existing round-robin -> ~1 Hz
  // per group. Sending only one frame at a time stays within the small
  // Serial1 TX FIFO.
  static uint16_t lastTick = 0;
  static uint8_t nextTelemetry = 0;
  static uint8_t sendChannels = 1;

  const uint16_t now = g_tmr10ms;
  if ((uint16_t)(now - lastTick) < 5U)
    return;

  lastTick = now;

  if (sendChannels)
    {
      telemOutSendRcChannels8();
    }
  else
    {
      switch (nextTelemetry)
        {
          case 0:
            telemOutSendLink();
            break;

          case 1:
            telemOutSendBattery();
            break;

          case 2:
            telemOutSendVario();
            break;

          case 3:
            telemOutSendGps();
            break;

          case 4:
            telemOutSendBaroAltitude();
            break;

          case 5:
            telemOutSendRpm();
            break;

          case 6:
            telemOutSendTemperature();
            break;

          case 7:
            telemOutSendCells();
            break;

          case 8:
            telemOutSendRxBatt();
            break;

          default:
            // Give GPS a second slot because it carries several displayed values.
            telemOutSendGps();
            break;
        }

      if (++nextTelemetry >= 10U)
        nextTelemetry = 0;
    }

  sendChannels ^= 1U;
}

void telemOutStop()
{
#if defined(CPUM2560) && !defined(SIMU)
  BT_POWER_OFF();
#endif
}

void telemOutSend(const uint8_t *data, uint8_t len)
{
  if (!data || !len)
    return;

  BT_Ser_Print(data, len);
  BT_Ser_SendTxBuffer();
}
