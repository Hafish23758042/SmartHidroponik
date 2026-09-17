// =====================================================================
//  HydroController — ESP32-S3
//  Monitoring 6 sensor RS485 Modbus RTU + kendali 4 kanal relay.
//
//  Prinsip perancangan:
//    1. Kendali berjalan penuh tanpa jaringan. Wi-Fi hanya pelaporan.
//    2. Keselamatan diperiksa SETIAP loop, terlepas dari logika kendali.
//    3. Kondisi awal = kondisi aman (semua relay OFF).
//    4. Nilai sensor adalah klaim yang harus dibuktikan, bukan fakta.
// =====================================================================

#include "config.h"
#include "types.h"
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <esp_task_wdt.h>
#include <math.h>

// Server Telnet untuk pemantauan Serial Monitor via TCP
WiFiServer telnetServer(TELNET_PORT);
WiFiClient telnetClient;

// Klien MQTT
WiFiClient netClient;
PubSubClient mqtt(netClient);

// =====================================================================
//  1. MODBUS RTU MASTER
// =====================================================================

HardwareSerial RS485(1);

// Deklarasi maju (extern): variabel-variabel ini baru DIDEFINISIKAN
// lebih jauh ke bawah berkas ini (dekat logika sensor/relay/manual
// masing-masing), tetapi blok DWIN di bawah ini memakainya lebih dulu.
// Tanpa extern di sini, kompiler C++ tidak mengenalinya pada titik
// pemakaian, karena (berbeda dari prototipe fungsi) Arduino IDE tidak
// menyisipkan deklarasi maju otomatis untuk variabel global.
extern float airT, airRh, luxV;
extern float ecV, tdsV, phV, wTempV;
extern float distV, levelPct;
extern bool relayState[4];
extern bool manualOn[4];
extern bool manualHold[4];
extern uint32_t manualSince[4];
extern const SensorDef SENSORS[SENSOR_COUNT];

enum DwinRegIndex {
  DWIN_IDX_AIR_T = 0,
  DWIN_IDX_AIR_RH = 1,
  DWIN_IDX_WTEMP = 2,
  DWIN_IDX_EC = 3,
  DWIN_IDX_TDS = 4,
  DWIN_IDX_DISTANCE = 5,
  DWIN_IDX_LEVEL = 6,
  DWIN_IDX_PH = 7,
  DWIN_IDX_LUX = 8,
  DWIN_IDX_ORP = 9,
  DWIN_IDX_PH_PROBE_TEMP = 10,
  DWIN_IDX_RELAY_PUMP_NUTRISI = 11,
  DWIN_IDX_RELAY_EXHAUST_FAN = 12,
  DWIN_IDX_RELAY_MISTING = 13,
  DWIN_IDX_RELAY_POMPA_UTAMA = 14,
  DWIN_IDX_COUNT = 15
};

const uint16_t dwinAddrMap[DWIN_IDX_COUNT] = {
    0x5100, 0x5101, 0x5102, 0x5103, 0x5104, 0x5105, 0x5106, 0x5107,
    0x5108, 0x5109, 0x510A, 0x5120, 0x5121, 0x5122, 0x5123};

uint16_t dwinRegs[DWIN_IDX_COUNT] = {0};

int dwinIndexOfAddr(uint16_t addr) {
  for (uint8_t i = 0; i < DWIN_IDX_COUNT; i++) {
    if (dwinAddrMap[i] == addr)
      return i;
  }
  return -1;
}

void updateDwinRegisters() {
  dwinRegs[DWIN_IDX_AIR_T] = isnan(airT) ? 0 : (uint16_t)lroundf(airT * 10.0f);
  dwinRegs[DWIN_IDX_AIR_RH] =
      isnan(airRh) ? 0 : (uint16_t)lroundf(airRh * 10.0f);
  dwinRegs[DWIN_IDX_WTEMP] =
      isnan(wTempV) ? 0 : (uint16_t)lroundf(wTempV * 100.0f);
  dwinRegs[DWIN_IDX_EC] = isnan(ecV) ? 0 : (uint16_t)lroundf(ecV);
  dwinRegs[DWIN_IDX_TDS] = isnan(tdsV) ? 0 : (uint16_t)lroundf(tdsV);
  dwinRegs[DWIN_IDX_DISTANCE] = isnan(distV) ? 0 : (uint16_t)lroundf(distV);
  dwinRegs[DWIN_IDX_LEVEL] =
      isnan(levelPct) ? 0 : (uint16_t)lroundf(levelPct * 10.0f);
  dwinRegs[DWIN_IDX_PH] = isnan(phV) ? 0 : (uint16_t)lroundf(phV * 100.0f);
  dwinRegs[DWIN_IDX_LUX] = isnan(luxV) ? 0 : (uint16_t)lroundf(luxV);
  dwinRegs[DWIN_IDX_ORP] = 0;
  dwinRegs[DWIN_IDX_PH_PROBE_TEMP] = 0;

  // Sinkronkan status relay ke register DWIN sesuai daftar VP address yang
  // dipakai layar. Ini memetakan satu relay ke satu channel aktual, meski
  // beberapa alias DWIN mengarah ke channel yang sama untuk kompatibilitas UI.
  dwinRegs[DWIN_IDX_RELAY_PUMP_NUTRISI] = relayState[R_PUMP] ? 1 : 0;
  dwinRegs[DWIN_IDX_RELAY_EXHAUST_FAN] = relayState[R_FAN] ? 1 : 0;
  dwinRegs[DWIN_IDX_RELAY_MISTING] = relayState[R_MIST] ? 1 : 0;
  dwinRegs[DWIN_IDX_RELAY_POMPA_UTAMA] = relayState[R_PUMP_UTAMA] ? 1 : 0;
}

static uint8_t dwinRelayChannelOfAddr(uint16_t addr) {
  switch (addr) {
  case 0x5120:
    return R_PUMP; // nutrisi / pompa nutrisi
  case 0x5121:
    return R_FAN;
  case 0x5122:
    return R_MIST;
  case 0x5123:
    return R_PUMP_UTAMA; // Pompa utama
  default:
    return 255;
  }
}

void dwinWriteHoldingReg(uint16_t addr, uint16_t value) {
  int idx = dwinIndexOfAddr(addr);
  if (idx < 0)
    return;

  uint8_t ch = dwinRelayChannelOfAddr(addr);
  if (ch != 255) {
    const bool on = value != 0;
    if (on && !manualAllowed(ch, true)) {
      publishEvent("dwin_manual_denied", relayName[ch]);
      return;
    }
    manualOn[ch] = on;
    manualHold[ch] = true;
    manualSince[ch] = millis();
    setRelay(ch, on);
    publishEvent(on ? "dwin_manual_on" : "dwin_manual_off", relayName[ch]);
    dwinRegs[idx] = on ? 1 : 0;
    return;
  }

  dwinRegs[idx] = value;
}

uint16_t modbusCRC(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1);
  }
  return crc;
}

void rs485Tx(bool on) {
  if (RS485_DE_PIN >= 0)
    digitalWrite(RS485_DE_PIN, on ? HIGH : LOW);
}

// Statistik bus untuk alarm W15
static uint32_t busTx = 0, busErr = 0;

// Membaca `qty` register. Mengembalikan true bila frame valid.
bool modbusRead(uint8_t id, uint8_t fc, uint16_t addr, uint8_t qty,
                uint16_t *out) {
  uint8_t req[8];
  req[0] = id;
  req[1] = fc;
  req[2] = addr >> 8;
  req[3] = addr & 0xFF;
  req[4] = 0;
  req[5] = qty;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485.available())
    RS485.read(); // buang sisa frame lama

  busTx++;
  rs485Tx(true);
  delayMicroseconds(100);
  RS485.write(req, 8);
  RS485.flush();
  delayMicroseconds(1200); // jeda tuntas stop bit UART pada bus RS485
  rs485Tx(false);

  const uint8_t expect = 5 + qty * 2; // id fc len data.. crc crc
  uint8_t resp[64];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (n < expect && (millis() - t0) < MODBUS_TIMEOUT_MS) {
    if (RS485.available()) {
      resp[n++] = RS485.read();
      // Deteksi dini frame Exception Modbus (fc | 0x80): panjang respon hanya 5
      // byte
      if (n == 2 && (resp[1] & 0x80)) {
        break;
      }
    }
  }

  // Tangani exception response (5 byte: id, fc|80, exception_code, crc_l,
  // crc_h)
  if (n >= 2 && (resp[1] & 0x80)) {
    while (n < 5 && (millis() - t0) < MODBUS_TIMEOUT_MS) {
      if (RS485.available())
        resp[n++] = RS485.read();
    }
    busErr++;
    return false;
  }

  if (n < expect) {
    busErr++;
    return false;
  }
  if (resp[0] != id || resp[1] != fc) {
    busErr++;
    return false;
  }
  if (resp[2] != qty * 2) {
    busErr++;
    return false;
  }

  uint16_t rcrc = modbusCRC(resp, expect - 2);
  if ((rcrc & 0xFF) != resp[expect - 2] || (rcrc >> 8) != resp[expect - 1]) {
    busErr++;
    return false;
  }

  for (uint8_t i = 0; i < qty; i++)
    out[i] = ((uint16_t)resp[3 + i * 2] << 8) | resp[4 + i * 2];
  return true;
}

// Menulis 1 register (fc=0x06). Mengembalikan true bila sukses.
bool modbusWrite(uint8_t id, uint16_t addr, uint16_t val) {
  uint8_t req[8];
  req[0] = id;
  req[1] = 0x06;
  req[2] = addr >> 8;
  req[3] = addr & 0xFF;
  req[4] = val >> 8;
  req[5] = val & 0xFF;
  uint16_t crc = modbusCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = crc >> 8;

  while (RS485.available())
    RS485.read(); // buang sisa frame lama

  busTx++;
  rs485Tx(true);
  delayMicroseconds(100);
  RS485.write(req, 8);
  RS485.flush(); // WAJIB tunggu hingga transmisi selesai
  delayMicroseconds(1200);
  rs485Tx(false);

  const uint8_t expect = 8;
  uint8_t resp[16];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (n < expect && (millis() - t0) < MODBUS_TIMEOUT_MS) {
    if (RS485.available()) {
      resp[n++] = RS485.read();
      if (n == 2 && (resp[1] & 0x80))
        break;
    }
  }

  if (n >= 2 && (resp[1] & 0x80)) {
    while (n < 5 && (millis() - t0) < MODBUS_TIMEOUT_MS) {
      if (RS485.available())
        resp[n++] = RS485.read();
    }
    busErr++;
    return false;
  }

  if (n < expect) {
    busErr++;
    return false;
  }
  if (resp[0] != id || resp[1] != 0x06) {
    busErr++;
    return false;
  }

  return true;
}

// Menulis banyak register berurutan (fc=0x10). Mengembalikan true bila sukses.
bool modbusWriteMultiple(uint8_t id, uint16_t addr, uint8_t qty,
                         const uint16_t *vals) {
  uint8_t req[64];
  req[0] = id;
  req[1] = 0x10;
  req[2] = addr >> 8;
  req[3] = addr & 0xFF;
  req[4] = 0;
  req[5] = qty;
  req[6] = qty * 2;

  uint8_t idx = 7;
  for (uint8_t i = 0; i < qty; i++) {
    req[idx++] = vals[i] >> 8;
    req[idx++] = vals[i] & 0xFF;
  }

  uint16_t crc = modbusCRC(req, idx);
  req[idx++] = crc & 0xFF;
  req[idx++] = crc >> 8;

  while (RS485.available())
    RS485.read(); // buang sisa frame lama

  busTx++;
  rs485Tx(true);
  delayMicroseconds(100);
  RS485.write(req, idx);
  RS485.flush(); // WAJIB tunggu hingga transmisi selesai
  delayMicroseconds(1200);
  rs485Tx(false);

  const uint8_t expect = 8;
  uint8_t resp[16];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (n < expect && (millis() - t0) < MODBUS_TIMEOUT_MS) {
    if (RS485.available()) {
      resp[n++] = RS485.read();
      if (n == 2 && (resp[1] & 0x80))
        break;
    }
  }

  if (n >= 2 && (resp[1] & 0x80)) {
    while (n < 5 && (millis() - t0) < MODBUS_TIMEOUT_MS) {
      if (RS485.available())
        resp[n++] = RS485.read();
    }
    busErr++;
    return false;
  }

  if (n < expect) {
    busErr++;
    return false;
  }
  if (resp[0] != id || resp[1] != 0x10) {
    busErr++;
    return false;
  }

  return true;
}

static bool dwinSupportsFc10 = true;

// Menulis blok register ke DWIN dengan dukungan otomatis:
// Pertama mencoba FC 0x10 (Tembakan Massal). Bila DWIN menolak (Illegal
// Function), otomatis beralih permanen ke FC 0x06 (Write Single) satu per satu.
bool dwinWriteBlock(uint16_t startAddr, uint8_t qty, const uint16_t *vals) {
  if (dwinSupportsFc10) {
    if (modbusWriteMultiple(DWIN_SLAVE_ID, startAddr, qty, vals)) {
      return true;
    }
    dwinSupportsFc10 = false;
    Serial.println("[DWIN] Modbus FC 0x10 ditolak/gagal. Otomatis beralih ke "
                   "FC 0x06 (Write Single)...");
  }

  // Fallback: Kirim satu per satu menggunakan FC 0x06 (modbusWrite)
  bool allOk = true;
  for (uint8_t i = 0; i < qty; i++) {
    if (!modbusWrite(DWIN_SLAVE_ID, startAddr + i, vals[i])) {
      allOk = false;
    }
    if (i < qty - 1) {
      delay(MODBUS_GAP_MS);
    }
  }
  return allOk;
}

void syncDwinMaster() {
  static bool dwinBootResetDone = false;
  if (!dwinBootResetDone) {
    // Saat boot awal, paksa tulis status OFF (0) ke register tombol DWIN
    // (0x5120..0x5123) agar layar tidak mengirim status ON sisa sesi sebelumnya
    // yang dapat menyalakan relay.
    uint16_t offVals[4] = {0, 0, 0, 0};
    dwinWriteBlock(0x5120, 4, offVals);
    delay(MODBUS_GAP_MS);
    dwinBootResetDone = true;
    return; // Lewati pembacaan tombol di siklus pertama boot
  }

  // 1. PULL: Baca status tombol relay dari layar DWIN (Alamat 0x5120 s/d
  // 0x5123) Total 4 register. Kita gunakan fungsi 0x03 untuk menarik semuanya
  // sekaligus
  uint16_t relayVals[4];
  if (modbusRead(DWIN_SLAVE_ID, 0x03, 0x5120, 4, relayVals)) {
    for (int i = 0; i < 4; i++) {
      int dwinIdx = DWIN_IDX_RELAY_PUMP_NUTRISI + i;
      if (relayVals[i] != dwinRegs[dwinIdx]) {
        // Jika status di layar berbeda dari dwinRegs, berarti user memencet
        // tombol
        dwinWriteHoldingReg(0x5120 + i, relayVals[i]);
      }
    }
  }

  delay(MODBUS_GAP_MS);

  // 2. PUSH: Kirim data sensor ke layar DWIN (Alamat 0x5100, Qty 11)
  bool okSensor = dwinWriteBlock(0x5100, 11, &dwinRegs[0]);

  delay(MODBUS_GAP_MS);

  // 3. PUSH: Kirim status relay ke layar DWIN (Alamat 0x5120, Qty 4)
  bool okRelay = dwinWriteBlock(0x5120, 4, &dwinRegs[11]);

  // Log berkala ke Serial Monitor setiap 5 detik
  static uint32_t lastDwinLog = 0;
  if (millis() - lastDwinLog >= 5000UL) {
    lastDwinLog = millis();
    Serial.printf("[DWIN] Kirim ke Layar: Sensor=%s, Relay=%s (Metode: %s)\n",
                  okSensor ? "OK" : "GAGAL", okRelay ? "OK" : "GAGAL",
                  dwinSupportsFc10 ? "FC 0x10 Multiple" : "FC 0x06 Single");
  }
}

bool modbusPing(uint8_t id, uint16_t addr = 0x0000) {
  uint16_t dummy = 0;
  return modbusRead(id, 0x03, addr, 1, &dummy);
}

void scanSlaveIds() {
  Serial.println("\n=== SCAN MODBUS SLAVE ID 1..8 ===");
  telnetPrintf("\r\n=== SCAN MODBUS SLAVE ID 1..8 ===\r\n");
  for (uint8_t id = 1; id <= 8; id++) {
    bool online = false;

    // Beberapa sensor tidak menjawab pada alamat 0x0000; mereka hanya valid
    // pada register yang dipakai oleh konfigurasi sensor. Jadi scan harus
    // mengecek alamat nyata, bukan cuma "ping 0000" generik.
    for (uint8_t k = 0; k < SENSOR_COUNT; k++) {
      if (SENSORS[k].id != id)
        continue;
      online = modbusPing(id, SENSORS[k].addr);
      break;
    }

    // fallback untuk DWIN HMI: register DWIN berada di 0x5120 & 0x5100
    if (!online && id == DWIN_SLAVE_ID) {
      online = modbusPing(id, 0x5120);
      if (!online)
        online = modbusPing(id, 0x5100);
      if (!online)
        online = modbusPing(id, 5120);
      if (!online)
        online = modbusPing(id, 5100);
    }

    // fallback: ping umum tetap berguna untuk perangkat yang memang valid di
    // 0x0000
    if (!online)
      online = modbusPing(id, 0x0000);

    Serial.print("ID ");
    Serial.print(id);
    if (id == DWIN_SLAVE_ID) {
      Serial.print(" (DWIN HMI)");
    }
    Serial.print(" -> ");
    Serial.println(online ? "ONLINE" : "OFFLINE");
    if (id == DWIN_SLAVE_ID) {
      telnetPrintf("ID %d (DWIN HMI) -> %s\r\n", id,
                   online ? "ONLINE" : "OFFLINE");
    } else {
      telnetPrintf("ID %d -> %s\r\n", id, online ? "ONLINE" : "OFFLINE");
    }
    delay(MODBUS_GAP_MS); // Jeda diam agar bus RS485 tenang sebelum ping node
                          // berikutnya
  }
  Serial.println("=== SCAN SELESAI ===\n");
  telnetPrintf("=== SCAN SELESAI ===\r\n\r\n");
}

void handleSerialCommands() {
  static String input;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      input.trim();
      if (input.length() > 0) {
        String cmd = input;
        cmd.toLowerCase();
        if (cmd == "scan" || cmd == "scanmodbus" || cmd == "slave") {
          scanSlaveIds();
        } else {
          Serial.println("Perintah tersedia: SCAN");
        }
      }
      input = "";
    } else {
      input += c;
    }
  }
}

float s16(uint16_t v) { return (float)(int16_t)v; }

// =====================================================================
//  3. NODE SENSOR
// =====================================================================

const SensorDef SENSORS[SENSOR_COUNT] = {
    {ID_ASAIR, 3, 0, 3},   // suhu / RH  (peta register BELUM diverifikasi)
    {ID_LUX, 3, 0, 4},     // RH, suhu, lux hi, lux lo
    {ID_CWT03S, 4, 0, 2},  // suhu / RH  (input register)
    {ID_ECTDS, 3, 0, 5},   // suhu, -, EC, salinitas, TDS
    {ID_PH, 3, 0, 2},      // pH, suhu air
    {ID_CWTHXXS, 3, 0, 2}, // RH, suhu
    {ID_LEVEL, 3, 52, 1},  // WT53R-485: 0x34 = jarak (mm), 1 register
};

NodeState node[SENSOR_COUNT];

// Kanal terfilter
Median fAirT[4], fAirRh[4]; // 0=ASAIR 1=LUX 2=CWT03S 3=CWTHXXS
Median fLux, fEC, fPH, fWtEC, fWtPH, fDist;

bool okAirT[4] = {false, false, false, false};
bool okAirRh[4] = {false, false, false, false};
bool okLux = false, okEC = false, okPH = false, okWtEC = false, okWtPH = false;
bool okDist = false;

// Nilai turunan yang dipakai logika kendali
float airT = NAN, airRh = NAN, luxV = NAN;
float ecV = NAN, tdsV = NAN, phV = NAN, wTempV = NAN;
float distV = NAN, levelPct = NAN;
bool airOk = false, ecOk = false, phOk = false, wTempOk = false,
     levelOk = false;
bool levelCritLatch = false; // histeresis level kritis

// Deteksi laju perubahan EC (C02)
float ecPrev = NAN;
uint8_t ecRateStrikes = 0;

// -------- pembacaan satu node --------
void decodeNode(uint8_t k, uint16_t *r) {
  switch (SENSORS[k].id) {

  case ID_ASAIR: {
    // CATATAN: urutan RH/suhu pada AGH3485 belum dikonfirmasi.
    // Verifikasi dengan menghembuskan napas ke sensor: kanal yang
    // naik cepat adalah kelembapan. Tukar bila perlu.
    float rh = r[0] / 10.0f;
    float t = s16(r[1]) / 10.0f;
    if (rh >= RH_VALID_MIN && rh <= RH_VALID_MAX) {
      fAirRh[0].push(rh);
      okAirRh[0] = true;
    }
    if (t >= ATEMP_VALID_MIN && t <= ATEMP_VALID_MAX) {
      fAirT[0].push(t);
      okAirT[0] = true;
    }
    break;
  }

  case ID_LUX: {
    float rh = r[0] / 10.0f;
    float t = s16(r[1]) / 10.0f;
    if (rh >= RH_VALID_MIN && rh <= RH_VALID_MAX) {
      fAirRh[1].push(rh);
      okAirRh[1] = true;
    }
    if (t >= ATEMP_VALID_MIN && t <= ATEMP_VALID_MAX) {
      fAirT[1].push(t);
      okAirT[1] = true;
    }
    // Peta lux BELUM dipastikan: 40003/40004 sebagai kata hi/lo, atau
    // 40003 langsung berisi nilai lux. Heuristik di bawah menoleransi
    // keduanya sampai pengujian senter/tangan memastikannya.
    uint32_t lx = ((uint32_t)r[2] << 16) | r[3];
    if (lx > 200000UL)
      lx = r[2];
    fLux.push((float)lx);
    okLux = true;
    break;
  }

  case ID_CWT03S: {
    float rh = r[0] / 10.0f;
    float t = s16(r[1]) / 10.0f;
    if (rh >= RH_VALID_MIN && rh <= RH_VALID_MAX) {
      fAirRh[2].push(rh);
      okAirRh[2] = true;
    }
    if (t >= ATEMP_VALID_MIN && t <= ATEMP_VALID_MAX) {
      fAirT[2].push(t);
      okAirT[2] = true;
    }
    break;
  }

  case ID_ECTDS: {
    float t = s16(r[0]) / 100.0f; // perhatikan: dibagi 100
    float ec = (float)r[2];
    if (t >= WTEMP_VALID_MIN && t <= WTEMP_VALID_MAX) {
      fWtEC.push(t);
      okWtEC = true;
    }

    // Uji laju perubahan sebelum nilai diterima (C02).
    bool rateOk = true;
    if (!isnan(ecPrev) && fabsf(ec - ecPrev) > EC_RATE_MAX) {
      rateOk = false;
      if (ecRateStrikes < 255)
        ecRateStrikes++;
    } else {
      ecRateStrikes = 0;
    }
    ecPrev = ec;

    if (rateOk && ec >= EC_VALID_MIN && ec <= EC_VALID_MAX) {
      fEC.push(ec);
      okEC = true;
    } else if (ec < EC_VALID_MIN || ec > EC_VALID_MAX) {
      okEC = false; // di luar rentang: tolak, jangan filter
    }
    break;
  }

  case ID_PH: {
    float ph = r[0] / 100.0f;
    float t = s16(r[1]) / 10.0f;
    if (ph >= PH_VALID_MIN && ph <= PH_VALID_MAX) {
      fPH.push(ph);
      okPH = true;
    } else
      okPH = false;
    if (t >= WTEMP_VALID_MIN && t <= WTEMP_VALID_MAX) {
      fWtPH.push(t);
      okWtPH = true;
    }
    break;
  }

  case ID_LEVEL: {
    // Nilai langsung dalam milimeter, tanpa pembagi.
    // Permukaan air yang beriak memantulkan laser secara tak menentu,
    // sehingga filter median di sini bukan sekadar penghalus.
    float d = (float)r[0];
    if (d >= DIST_VALID_MIN && d <= DIST_VALID_MAX) {
      fDist.push(d);
      okDist = true;
    } else
      okDist = false;
    break;
  }

  case ID_CWTHXXS: {
    float rh = r[0] / 10.0f;
    float t = s16(r[1]) / 10.0f;
    if (rh >= RH_VALID_MIN && rh <= RH_VALID_MAX) {
      fAirRh[3].push(rh);
      okAirRh[3] = true;
    }
    if (t >= ATEMP_VALID_MIN && t <= ATEMP_VALID_MAX) {
      fAirT[3].push(t);
      okAirT[3] = true;
    }
    break;
  }
  }
}

// Median dari sumber udara yang tersedia (ID 1/2/3/6 saling mengoreksi).
bool medianOfAir(Median *f, bool *ok, float &out) {
  float v[4];
  uint8_t n = 0;
  for (uint8_t i = 0; i < 4; i++)
    if (ok[i] && f[i].ready)
      v[n++] = f[i].value;
  if (n == 0)
    return false;
  for (uint8_t i = 1; i < n; i++) {
    float k = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > k) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = k;
  }
  out = v[n / 2];
  return true;
}

void pollOne(uint8_t k) {
  uint16_t r[8] = {0};
  bool ok = modbusRead(SENSORS[k].id, SENSORS[k].fc, SENSORS[k].addr,
                       SENSORS[k].qty, r);

  if (!ok) {
    if (node[k].failStreak < 255)
      node[k].failStreak++;
    if (node[k].failStreak >= COMM_FAIL_LIMIT)
      node[k].online = false;
    return;
  }
  node[k].failStreak = 0;
  node[k].online = true;

  // Deteksi nilai beku (C07): perangkat menjawab tetapi isinya tak berubah.
  bool same = true;
  for (uint8_t i = 0; i < SENSORS[k].qty; i++)
    if (r[i] != node[k].lastRaw[i]) {
      same = false;
      break;
    }
  if (same) {
    if (node[k].stuckCount < 65535)
      node[k].stuckCount++;
  } else
    node[k].stuckCount = 0;
  for (uint8_t i = 0; i < SENSORS[k].qty; i++)
    node[k].lastRaw[i] = r[i];

  if (node[k].stuckCount >= STUCK_LIMIT)
    return; // jangan pakai data beku

  decodeNode(k, r);
}

void updateDerived() {
  airOk = medianOfAir(fAirT, okAirT, airT);
  bool rhOk = medianOfAir(fAirRh, okAirRh, airRh);
  if (!rhOk)
    airRh = NAN;

  luxV = (okLux && fLux.ready) ? fLux.value : NAN;

  ecOk =
      okEC && fEC.ready && node[3].online && node[3].stuckCount < STUCK_LIMIT;
  ecV = ecOk ? fEC.value : NAN;
  tdsV = ecOk ? ecV * TDS_FACTOR : NAN;

  phOk = okPH && fPH.ready && node[4].online;
  phV = phOk ? fPH.value : NAN;

  // --- level tandon ---
  levelOk = okDist && fDist.ready && node[6].online &&
            node[6].stuckCount < STUCK_LIMIT;
  if (levelOk) {
    distV = fDist.value;
    // Jarak mengecil saat terisi -> peta terbalik.
    levelPct =
        100.0f * (LEVEL_EMPTY_MM - distV) / (LEVEL_EMPTY_MM - LEVEL_FULL_MM);
    if (levelPct < 0.0f)
      levelPct = 0.0f;
    if (levelPct > 100.0f)
      levelPct = 100.0f;

    // Histeresis: sekali jatuh di bawah kritis, baru pulih setelah
    // melewati ambang pemulihan yang lebih tinggi.
    if (!levelCritLatch && levelPct < LEVEL_CRIT_PCT)
      levelCritLatch = true;
    if (levelCritLatch && levelPct > LEVEL_RESUME_PCT)
      levelCritLatch = false;
  } else {
    distV = NAN;
    levelPct = NAN;
  }

  // Suhu air: ID 4 lebih presisi, ID 5 sebagai cadangan.
  if (okWtEC && fWtEC.ready) {
    wTempV = fWtEC.value;
    wTempOk = true;
  } else if (okWtPH && fWtPH.ready) {
    wTempV = fWtPH.value;
    wTempOk = true;
  } else {
    wTempV = NAN;
    wTempOk = false;
  }
}

// =====================================================================
//  4. RELAY + PENGAMAN DURASI
// =====================================================================

bool relayState[4] = {false, false, false, false};
uint32_t relayOnSince[4] = {0, 0, 0, 0};
bool relayLatch[4] = {false, false, false, false};
uint32_t relayTotalOn[4] = {0, 0, 0, 0};

void publishEvent(const char *kind, const char *detail); // fwd

void applyPin(int ch, bool on) {
  digitalWrite(relayPin[ch], (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void setRelay(uint8_t ch, bool on) {
  if (ch >= 4)
    return;
  if (!relayEnabled[ch])
    on = false; // kanal nonaktif dipaksa mati
  if (relayLatch[ch])
    on = false; // terkunci sampai reset manual
  if (relayState[ch] == on)
    return;

  relayState[ch] = on;
  applyPin(ch, on);
  if (on)
    relayOnSince[ch] = millis();
  else {
    if (relayOnSince[ch])
      relayTotalOn[ch] += millis() - relayOnSince[ch];
    relayOnSince[ch] = 0;
  }
  publishEvent("relay", relayName[ch]);
  publishRelayState(ch);
}

void publishRelayState(uint8_t idx) {
  if (idx >= 4 || !mqtt.connected())
    return;
  char topic[80];
  snprintf(topic, sizeof(topic), "%s/%d/state", TOPIC_RELAY_BASE, idx + 1);
  mqtt.publish(topic, relayState[idx] ? "ON" : "OFF", true); // retained
}

void publishAllRelayState() {
  if (!mqtt.connected())
    return;
  char topic[80];
  char payload[160];
  snprintf(topic, sizeof(topic), "%s/state", TOPIC_RELAY_BASE);
  snprintf(payload, sizeof(payload),
           "{\"relay1\":\"%s\",\"relay2\":\"%s\",\"relay3\":\"%s\",\"relay4\":"
           "\"%s\",\"rssi\":%d}",
           relayState[0] ? "ON" : "OFF", relayState[1] ? "ON" : "OFF",
           relayState[2] ? "ON" : "OFF", relayState[3] ? "ON" : "OFF",
           WiFi.RSSI());
  mqtt.publish(topic, payload, true);
  for (uint8_t i = 0; i < 4; i++)
    publishRelayState(i);
}

void forceOff(uint8_t ch) {
  relayState[ch] = false;
  applyPin(ch, false);
  if (relayOnSince[ch])
    relayTotalOn[ch] += millis() - relayOnSince[ch];
  relayOnSince[ch] = 0;
  publishRelayState(ch);
}

// Dipanggil pada SETIAP loop. Ini adalah lapisan yang berada di bawah
// logika kendali: kesalahan logika di atas tetap tidak bisa melanggarnya.
bool guardTripped = false;

void relayGuard() {
  for (uint8_t i = 0; i < 4; i++) {
    // Jangan memakai relayOnSince == 0 sebagai penanda "mati": millis()
    // bernilai 0 tepat setelah boot, sehingga relay yang menyala pada
    // milidetik pertama tidak akan pernah dijaga. relayState sudah cukup.
    if (!relayState[i] || !RELAY_MAX_ON_MS[i])
      continue;
    if (millis() - relayOnSince[i] > RELAY_MAX_ON_MS[i]) {
      forceOff(i);
      relayLatch[i] = true;
      guardTripped = true;
      publishEvent("guard_trip", relayName[i]);
    }
  }
}

bool floatHigh() {
  if (FLOAT_HIGH_PIN < 0)
    return false;
  int v = digitalRead(FLOAT_HIGH_PIN);
  return FLOAT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// =====================================================================
//  5. ALARM
// =====================================================================

const AlarmDef ALARMS[ALARM_COUNT] = {
    {"C01a", true}, {"C01b", true},  {"C02", true},  {"C03", true},
    {"C04", true},  {"C05", true},   {"C06", true},  {"C07", true},
    {"C08", true},  {"C09", true},   {"C10", true},  {"W01", false},
    {"W02", false}, {"W02b", false}, {"W03", false}, {"W04", false},
    {"W06", false}, {"W07", false},  {"W08", false}, {"W09", false},
    {"W10", false}, {"W11", false},  {"W12", false}, {"W13", false},
    {"W15", false}, {"W16", false},  {"W20", false}};

AlarmState alarmSt[ALARM_COUNT];

bool maintenanceMode = false; // bungkam alarm saat kalibrasi

void publishAlarm(uint8_t i, bool on); // fwd

// Konfirmasi berulang: gangguan sesaat tidak boleh memicu alarm.
void evalAlarm(uint8_t i, bool cond) {
  if (maintenanceMode)
    return;
  AlarmState &a = alarmSt[i];
  const uint8_t need =
      ALARMS[i].critical ? ALARM_CONFIRM_CRIT : ALARM_CONFIRM_WARN;

  if (cond) {
    if (a.confirm < need)
      a.confirm++;
    if (a.confirm >= need && !a.active) {
      a.active = true;
      a.since = millis();
      if (ALARMS[i].critical)
        a.latched = true;
      publishAlarm(i, true);
    } else if (a.active && millis() - a.lastPub > ALARM_REPUBLISH_MS) {
      publishAlarm(i, true);
    }
  } else {
    a.confirm = 0;
    if (a.active && !a.latched) {
      a.active = false;
      publishAlarm(i, false);
    }
  }
}

bool isActive(uint8_t i) { return alarmSt[i].active; }

bool anyNodeOffline() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    if (node[i].failStreak >= COMM_FAIL_LIMIT)
      return true;
  return false;
}

// Kesehatan satu node tertentu. Dipakai agar penguncian aktuator
// hanya dipicu oleh sensor yang benar-benar relevan.
bool nodeStuck(uint8_t idx) {
  return idx < SENSOR_COUNT && node[idx].stuckCount >= STUCK_LIMIT;
}

bool nodeBad(uint8_t idx) {
  if (idx >= SENSOR_COUNT)
    return true;
  return nodeStuck(idx) || node[idx].failStreak >= COMM_FAIL_LIMIT;
}

// Daftar kode alarm aktif, untuk telemetri.
uint8_t activeAlarmList(char *dst, size_t n) {
  dst[0] = 0;
  uint8_t cnt = 0;
  for (uint8_t i = 0; i < ALARM_COUNT; i++) {
    if (!alarmSt[i].active)
      continue;
    cnt++;
    size_t used = strlen(dst);
    size_t need = strlen(ALARMS[i].code) + (used ? 1 : 0) + 1;
    if (used + need >= n)
      continue;
    if (used)
      strcat(dst, ",");
    strcat(dst, ALARMS[i].code);
  }
  return cnt;
}

bool anyNodeStuck() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    if (node[i].stuckCount >= STUCK_LIMIT)
      return true;
  return false;
}

uint8_t doseCount = 0; // dideklarasikan lebih awal untuk C03

void evaluateAlarms() {
  // --- kritis ---
  evalAlarm(C01A, ecOk ? false
                       : (okEC == false && node[3].online && !isnan(ecPrev) &&
                          ecPrev < EC_CRIT_LOW));
  evalAlarm(C01B, !isnan(ecPrev) && ecPrev > EC_CRIT_HIGH);
  evalAlarm(C02, ecRateStrikes >= EC_RATE_STRIKES);
  evalAlarm(C03, doseCount >= DOSE_MAX_PER_DAY);
  evalAlarm(C04, guardTripped);
  evalAlarm(C05, wTempOk && wTempV > WTEMP_CRIT_HIGH);
  evalAlarm(C06, anyNodeOffline());
  evalAlarm(C07, anyNodeStuck());
  evalAlarm(C08, floatHigh());
  evalAlarm(C09, phOk && (phV < PH_CRIT_LOW || phV > PH_CRIT_HIGH));
  // C10: tandon nyaris kosong. Menambah pekatan ke air yang sedikit
  // menaikkan EC jauh lebih cepat daripada perhitungan dosis normal.
  evalAlarm(C10, levelOk && levelCritLatch);

  // --- peringatan ---
  evalAlarm(W01, ecOk && ecV < EC_WARN_LOW);
  evalAlarm(W02, ecOk && ecV > EC_WARN_HIGH);
  evalAlarm(W02B, ecOk && ecV > EC_WARN_HIGH2);
  evalAlarm(W03, phOk && phV < PH_WARN_LOW);
  evalAlarm(W04, phOk && phV > PH_WARN_HIGH);
  evalAlarm(W06, wTempOk && wTempV > WTEMP_WARN_HIGH);
  evalAlarm(W07, wTempOk && wTempV < WTEMP_WARN_LOW);

  // W08: dua sensor suhu air menyimpang -> indikasi sirkulasi mati.
  bool diverge = (okWtEC && fWtEC.ready && okWtPH && fWtPH.ready) &&
                 fabsf(fWtEC.value - fWtPH.value) > WTEMP_DIVERGE;
  evalAlarm(W08, diverge);

  evalAlarm(W09, airOk && airT > ATEMP_WARN_HIGH);
  evalAlarm(W10, airOk && airT > ATEMP_WARN_HIGH2);
  evalAlarm(W11, !isnan(airRh) && airRh > RH_WARN_HIGH);
  evalAlarm(W12, !isnan(airRh) && airRh < RH_WARN_LOW);

  // W13: satu sensor udara menyimpang dari kelompoknya -> sensor bermasalah.
  bool airDiv = false;
  if (airOk) {
    for (uint8_t i = 0; i < 4; i++)
      if (okAirT[i] && fAirT[i].ready &&
          fabsf(fAirT[i].value - airT) > AIR_DIVERGE_T)
        airDiv = true;
  }
  if (!isnan(airRh)) {
    for (uint8_t i = 0; i < 4; i++)
      if (okAirRh[i] && fAirRh[i].ready &&
          fabsf(fAirRh[i].value - airRh) > AIR_DIVERGE_RH)
        airDiv = true;
  }
  evalAlarm(W13, airDiv);

  float errPct = busTx ? (100.0f * busErr / busTx) : 0.0f;
  evalAlarm(W20, levelOk && levelPct < LEVEL_WARN_PCT);
  evalAlarm(W15, busTx > 100 && errPct > RS485_ERR_PCT);
  evalAlarm(W16, WiFi.status() != WL_CONNECTED);
}

// =====================================================================
//  5b. KENDALI MANUAL LEWAT MQTT
//
//  Perintah manual TIDAK memintas satu pun pengaman. Semuanya tetap
//  melewati setRelay(), sehingga relay guard, tanda kanal nonaktif,
//  dan latch kesalahan tetap berlaku. Perintah juga kedaluwarsa
//  sendiri agar tidak ada aktuator yang menyala tanpa batas.
// =====================================================================

bool manualOn[4] = {false, false, false, false};
bool manualHold[4] = {false, false, false, false};
uint32_t manualSince[4] = {0, 0, 0, 0};

bool manualActive(uint8_t ch) {
  // Penanda aktif berupa boolean tersendiri, bukan "timestamp bukan nol".
  if (ch >= 4 || !manualHold[ch])
    return false;
  if (millis() - manualSince[ch] >= MANUAL_TIMEOUT_MS) {
    manualHold[ch] = false;
    publishEvent("manual_expire", relayName[ch]);
    return false;
  }
  return true;
}

void manualClear(uint8_t ch) {
  if (ch < 4)
    manualHold[ch] = false;
}

// Menyalakan butuh izin; mematikan selalu boleh.
bool manualAllowed(uint8_t ch, bool on) {
  if (!on)
    return true;
  if (ch >= 4 || !relayEnabled[ch] || relayLatch[ch])
    return false;
  if (ch == R_PUMP) {
    // Interlock pompa tetap berlaku untuk perintah manual.
    if (isActive(C01A) || isActive(C01B) || isActive(C08) || isActive(C10))
      return false;
    if (floatHigh())
      return false;
  }
  return true;
}

void applyManual() {
  for (uint8_t i = 0; i < 4; i++) {
    if (!manualActive(i))
      continue;
    if (manualOn[i] && !manualAllowed(i, true)) {
      manualOn[i] = false;
      manualClear(i);
      publishEvent("manual_denied", relayName[i]);
    }
    setRelay(i, manualOn[i]);
  }
}

// =====================================================================
//  6. MESIN KEADAAN PENDOSISAN
// =====================================================================

DoseState doseState = D_IDLE;

uint32_t doseT0 = 0;          // awal dosis / awal jeda campur
uint32_t doseWindowStart = 0; // jendela 24 jam

const char *doseStateName() {
  switch (doseState) {
  case D_IDLE:
    return "idle";
  case D_DOSING:
    return "dosing";
  case D_MIXING:
    return "mixing";
  default:
    return "locked";
  }
}

const char *doseLockWhy = "";

void doseLock(const char *why) {
  doseLockWhy = why;
  doseState = D_LOCKED;
  setRelay(R_PUMP, false);
  publishEvent("dose_lock", why);
}

void controlDosing() {
  // Jendela harian bergulir. Untuk penanggalan sebenarnya gunakan NTP.
  if (millis() - doseWindowStart > DOSE_WINDOW_MS) {
    doseWindowStart = millis();
    doseCount = 0;
  }

  // Kondisi yang mengunci pendosisan, diperiksa lebih dulu.
  if (doseState != D_LOCKED) {
    // C07 bersifat global (ketujuh node). Mengunci pompa nutrisi karena
    // sensor suhu udara membeku tidak masuk akal, jadi di sini hanya
    // node EC — indeks 3 — yang diperiksa.
    if (isActive(C01A) || isActive(C01B) || isActive(C02) || nodeBad(3) ||
        isActive(C08) || isActive(C09) || isActive(C10)) {
      doseLock(isActive(C09) ? "ph_ekstrem"
               : nodeBad(3)  ? "sensor_ec"
                             : "alarm_kritis");
    } else if (relayLatch[R_PUMP]) {
      doseLock("relay_latch");
    } else if (doseCount >= DOSE_MAX_PER_DAY) {
      doseLock("plafon_harian");
    }
  }

  // Kanal di bawah kendali manual tidak disentuh logika otomatis.
  if (manualActive(R_PUMP))
    return;

  switch (doseState) {

  case D_IDLE:
    setRelay(R_PUMP, false);
    // Hanya mendosis bila data sahih DAN benar-benar di bawah ambang.
    if (ecOk && fEC.ready && ecV < EC_DOSE_START && !floatHigh() &&
        !(levelOk && levelCritLatch)) {
      doseState = D_DOSING;
      doseT0 = millis();
      doseCount++;
      setRelay(R_PUMP, true);
      publishEvent("dose_start", "");
    }
    break;

  case D_DOSING:
    if (!ecOk || floatHigh()) { // data hilang saat mendosis
      setRelay(R_PUMP, false);
      doseState = D_MIXING;
      doseT0 = millis();
      break;
    }
    if (millis() - doseT0 >= DOSE_MS || ecV >= EC_DOSE_STOP) {
      setRelay(R_PUMP, false);
      doseState = D_MIXING;
      doseT0 = millis();
      publishEvent("dose_end", "");
    }
    break;

  case D_MIXING:
    setRelay(R_PUMP, false);
    // Inti pencegahan overdosis: JANGAN mengevaluasi EC sebelum
    // larutan homogen, atau sistem akan mendosis berulang.
    if (millis() - doseT0 >= MIX_WAIT_MS)
      doseState = D_IDLE;
    break;

  case D_LOCKED:
    setRelay(R_PUMP, false);
    break; // hanya keluar via reset manual
  }
}

// =====================================================================
//  7. KENDALI IKLIM
//     Misting dan exhaust saling meniadakan bila aktif bersamaan.
//     RH menjadi penentu mana yang boleh bekerja.
// =====================================================================

uint32_t mistLastStart = 0;
bool climateHot = false; // status histeresis

void controlClimate() {
  const bool mistManual = manualActive(R_MIST);
  const bool fanManual = manualActive(R_FAN);

  if (!airOk) {
    if (!mistManual)
      setRelay(R_MIST, false);
    if (!fanManual)
      setRelay(R_FAN, false);
    return;
  }

  if (!climateHot && airT > CLIMATE_T_ON)
    climateHot = true;
  if (climateHot && airT < CLIMATE_T_OFF)
    climateHot = false;

  bool rhKnown = !isnan(airRh);

  // ---- misting ----
  bool mistWant = climateHot && rhKnown && airRh < MIST_RH_CEILING;
  if (mistManual) { /* dilewati */
  } else if (relayState[R_MIST]) {
    if (!mistWant || millis() - mistLastStart >= MIST_BURST_MS)
      setRelay(R_MIST, false);
  } else if (mistWant && !relayLatch[R_MIST]) {
    // siklus kerja: satu semburan per periode
    if (mistLastStart == 0 || millis() - mistLastStart >= MIST_PERIOD_MS) {
      mistLastStart = millis();
      setRelay(R_MIST, true);
    }
  }

  // ---- exhaust fan ----
  static bool fanRhLatch = false;
  if (rhKnown) {
    if (!fanRhLatch && airRh > FAN_RH_ON)
      fanRhLatch = true;
    if (fanRhLatch && airRh < FAN_RH_OFF)
      fanRhLatch = false;
  }
  // Fan tidak boleh menyala saat misting bekerja: kabut akan terbuang
  // sebelum sempat menguap dan mendinginkan.
  bool fanWant =
      (fanRhLatch || (climateHot && rhKnown && airRh >= MIST_RH_CEILING)) &&
      !relayState[R_MIST];
  if (!fanManual)
    setRelay(R_FAN, fanWant);
}

// =====================================================================
//  8. JARINGAN + MQTT
// =====================================================================

char topStatus[80], topTele[80], topAlarm[80], topEvent[80], topCmd[80];

Event evBuf[EVENT_BUF_SIZE];
uint8_t evHead = 0, evCount = 0;

void evPush(const char *kind, const char *detail) {
  Event &e = evBuf[evHead];
  strncpy(e.kind, kind, sizeof(e.kind) - 1);
  e.kind[sizeof(e.kind) - 1] = 0;
  strncpy(e.detail, detail, sizeof(e.detail) - 1);
  e.detail[sizeof(e.detail) - 1] = 0;
  e.ts = millis();
  evHead = (evHead + 1) % EVENT_BUF_SIZE;
  if (evCount < EVENT_BUF_SIZE)
    evCount++; // penuh -> terlama tertimpa
}

void publishEvent(const char *kind, const char *detail) {
  char pl[128];
  snprintf(pl, sizeof(pl), "{\"kind\":\"%s\",\"detail\":\"%s\",\"ts\":%lu}",
           kind, detail, (unsigned long)millis());
  if (mqtt.connected())
    mqtt.publish(topEvent, pl, false);
  else
    evPush(kind, detail);
  Serial.printf("[EVENT] %s %s\n", kind, detail);
  telnetPrintf("[EVENT] %s %s\r\n", kind, detail);
}

void publishAlarm(uint8_t i, bool on) {
  alarmSt[i].lastPub = millis();
  char pl[160];
  snprintf(pl, sizeof(pl),
           "{\"code\":\"%s\",\"state\":\"%s\",\"level\":\"%s\",\"ts\":%lu}",
           ALARMS[i].code, on ? "active" : "clear",
           ALARMS[i].critical ? "critical" : "warning",
           (unsigned long)millis());
  if (mqtt.connected())
    mqtt.publish(topAlarm, pl, false);
  else
    evPush("alarm", ALARMS[i].code);
  Serial.printf("[ALARM] %s %s\n", ALARMS[i].code, on ? "ACTIVE" : "clear");
  telnetPrintf("[ALARM] %s %s\r\n", ALARMS[i].code, on ? "ACTIVE" : "clear");
}

void flushEvents() {
  while (evCount && mqtt.connected()) {
    uint8_t idx = (evHead + EVENT_BUF_SIZE - evCount) % EVENT_BUF_SIZE;
    Event &e = evBuf[idx];
    char pl[128];
    snprintf(pl, sizeof(pl),
             "{\"kind\":\"%s\",\"detail\":\"%s\",\"ts\":%lu,\"buffered\":true}",
             e.kind, e.detail, (unsigned long)e.ts);
    if (!mqtt.publish(topEvent, pl, false))
      break;
    evCount--;
  }
}

// Nilai terakhir yang dikirim, untuk pengiriman berbasis perubahan.
float lastTds = NAN, lastPh = NAN, lastWt = NAN, lastAt = NAN, lastRh = NAN;

bool movedEnough() {
  if (!isnan(tdsV) && (isnan(lastTds) || fabsf(tdsV - lastTds) >= DELTA_TDS))
    return true;
  if (!isnan(phV) && (isnan(lastPh) || fabsf(phV - lastPh) >= DELTA_PH))
    return true;
  if (!isnan(wTempV) &&
      (isnan(lastWt) || fabsf(wTempV - lastWt) >= DELTA_WTEMP))
    return true;
  if (!isnan(airT) && (isnan(lastAt) || fabsf(airT - lastAt) >= DELTA_ATEMP))
    return true;
  if (!isnan(airRh) && (isnan(lastRh) || fabsf(airRh - lastRh) >= DELTA_RH))
    return true;
  return false;
}

void fmt(char *dst, size_t n, float v, uint8_t dec) {
  if (isnan(v))
    snprintf(dst, n, "null");
  else
    snprintf(dst, n, "%.*f", dec, v);
}

void publishTelemetry() {
  char sAt[12], sRh[12], sLx[14], sEc[12], sTds[12], sPh[12], sWt[12];
  char sD[12], sLv[12], sAl[120];
  fmt(sAt, sizeof(sAt), airT, 1);
  fmt(sRh, sizeof(sRh), airRh, 1);
  fmt(sLx, sizeof(sLx), luxV, 0);
  fmt(sEc, sizeof(sEc), ecV, 0);
  fmt(sTds, sizeof(sTds), tdsV, 0);
  fmt(sPh, sizeof(sPh), phV, 2);
  fmt(sWt, sizeof(sWt), wTempV, 2);
  fmt(sD, sizeof(sD), distV, 0);
  fmt(sLv, sizeof(sLv), levelPct, 1);
  uint8_t nAl = activeAlarmList(sAl, sizeof(sAl));

  char pl[720];
  snprintf(pl, sizeof(pl),
           "{\"air_t\":%s,\"air_rh\":%s,\"lux\":%s,\"ec\":%s,\"tds\":%s,"
           "\"ph\":%s,\"water_t\":%s,\"dist_mm\":%s,\"level_pct\":%s,"
           "\"dose_state\":\"%s\",\"dose_count\":%u,"
           "\"relay\":[%d,%d,%d,%d],\"manual\":[%d,%d,%d,%d],"
           "\"lock\":\"%s\",\"alarm_n\":%u,\"alarms\":\"%s\"}",
           sAt, sRh, sLx, sEc, sTds, sPh, sWt, sD, sLv, doseStateName(),
           doseCount, relayState[0], relayState[1], relayState[2],
           relayState[3], manualActive(0), manualActive(1), manualActive(2),
           manualActive(3), doseState == D_LOCKED ? doseLockWhy : "", nAl, sAl);

  if (mqtt.connected())
    mqtt.publish(topTele, pl, false);

  lastTds = tdsV;
  lastPh = phV;
  lastWt = wTempV;
  lastAt = airT;
  lastRh = airRh;
}

void publishHeartbeat() {
  float errPct = busTx ? (100.0f * busErr / busTx) : 0.0f;
  char pl[240];
  snprintf(pl, sizeof(pl),
           "{\"status\":\"online\",\"uptime_s\":%lu,\"rssi\":%d,\"heap\":%u,"
           "\"bus_tx\":%lu,\"bus_err\":%lu,\"bus_err_pct\":%.2f,\"maint\":%d}",
           (unsigned long)(millis() / 1000), WiFi.RSSI(),
           (unsigned)ESP.getFreeHeap(), (unsigned long)busTx,
           (unsigned long)busErr, errPct, maintenanceMode);
  if (mqtt.connected())
    mqtt.publish(topStatus, pl, true);
}

// ----------------- Telnet Serial Monitor -----------------
void setupTelnet() {
  telnetServer.begin();
  telnetServer.setNoDelay(true);
  Serial.printf("[TELNET] Server aktif di port %d\n", TELNET_PORT);
}

void loopTelnet() {
  if (telnetServer.hasClient()) {
    if (!telnetClient || !telnetClient.connected()) {
      if (telnetClient)
        telnetClient.stop();
      telnetClient = telnetServer.available();
      telnetClient.println("\n=== Selamat Datang di HydroController Serial "
                           "Monitor (Telnet) ===");
      telnetClient.println("Ketik SCAN untuk scan Modbus Slave ID\n");
    } else {
      // Tolak koneksi klien tambahan bila sudah ada 1 sesi aktif
      WiFiClient extra = telnetServer.available();
      extra.println("Koneksi ditolak: Sesi Telnet sudah aktif.");
      extra.stop();
    }
  }

  // Baca perintah dari Telnet client dan proses sama seperti Serial USB
  if (telnetClient && telnetClient.connected() && telnetClient.available()) {
    static String telnetInput;
    while (telnetClient.available()) {
      char c = telnetClient.read();
      if (c == '\n' || c == '\r') {
        telnetInput.trim();
        if (telnetInput.length() > 0) {
          String cmd = telnetInput;
          cmd.toLowerCase();
          if (cmd == "scan" || cmd == "scanmodbus" || cmd == "slave") {
            scanSlaveIds();
          } else {
            telnetPrintf("Perintah tersedia: SCAN\r\n");
          }
        }
        telnetInput = "";
      } else {
        telnetInput += c;
      }
    }
  }
}

void telnetPrint(const char *s) {
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(s);
  }
}

void telnetPrintf(const char *format, ...) {
  if (!telnetClient || !telnetClient.connected())
    return;
  char buf[256];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  telnetClient.print(buf);
}

// ----------------- ArduinoOTA Firmware Update -----------------
static bool otaInitialized = false;

void setupOta() {
  if (otaInitialized)
    return;

  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  if (strlen(OTA_PASSWORD) > 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    String type =
        (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    Serial.println("[OTA] Proses update dimulai: " + type);
    telnetPrintf("[OTA] Update dimulai (%s), sistem berhenti sementara...\r\n",
                 type.c_str());
    // Matikan seluruh relay demi keamanan sebelum flash firmware baru
    for (uint8_t i = 0; i < 4; i++) {
      forceOff(i);
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update selesai! Me-reboot ESP32...");
    telnetPrintf("\r\n[OTA] Update sukses! Rebooting...\r\n");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned int lastPct = 0;
    unsigned int pct = (progress / (total / 100));
    if (pct != lastPct && pct % 10 == 0) {
      lastPct = pct;
      Serial.printf("[OTA] Progress: %u%%\r\n", pct);
      telnetPrintf("[OTA] Progress: %u%%\r\n", pct);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Gagal");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Gagal");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Gagal");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Gagal");
    else if (error == OTA_END_ERROR)
      Serial.println("End Gagal");
  });

  ArduinoOTA.begin();
  otaInitialized = true;
  Serial.println("[OTA] ArduinoOTA siap (Port: 3232)");
}

void loopOta() {
  if (otaInitialized && WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }
}

void printSensorSerial() {
  char buf[160];
  snprintf(buf, sizeof(buf),
           "SENSOR airT=%.1f airRh=%.1f lux=%.0f ec=%.1f tds=%.1f ph=%.2f "
           "wTemp=%.2f dist=%.0f levelPct=%.1f\n",
           airT, airRh, luxV, ecV, tdsV, phV, wTempV, distV, levelPct);
  Serial.print(buf);
  telnetPrint(buf);
}

// Perintah masuk: reset kunci, mode pemeliharaan, serta kontrol relay manual.
void onMqtt(char *topic, byte *payload, unsigned int len) {
  char msg[32];
  unsigned int n = len < sizeof(msg) - 1 ? len : sizeof(msg) - 1;
  for (unsigned int i = 0; i < n; i++)
    msg[i] = toupper((char)payload[i]);
  msg[n] = 0;

  bool isOn = (strcmp(msg, "ON") == 0 || strcmp(msg, "1") == 0 ||
               strcmp(msg, "TRUE") == 0);
  bool isOff = (strcmp(msg, "OFF") == 0 || strcmp(msg, "0") == 0 ||
                strcmp(msg, "FALSE") == 0);
  bool isToggle = (strcmp(msg, "TOGGLE") == 0);

  // Helper lambda untuk menjalankan kontrol manual pada 1 kanal
  auto applyManualRelay = [](uint8_t ch, bool targetOn) {
    if (ch >= 4)
      return;
    if (targetOn && !manualAllowed(ch, true)) {
      publishEvent("manual_denied", relayName[ch]);
      return;
    }
    manualOn[ch] = targetOn;
    manualHold[ch] = true;
    manualSince[ch] = millis();
    setRelay(ch, targetOn);
    publishEvent(targetOn ? "manual_on" : "manual_off", relayName[ch]);
  };

  // 1. Topic .../relay/all (kontrol semua relay sekaligus)
  char tAll[80];
  snprintf(tAll, sizeof(tAll), "%s/all", TOPIC_RELAY_BASE);
  if (strcmp(topic, tAll) == 0) {
    for (uint8_t i = 0; i < 4; i++) {
      if (isOn)
        applyManualRelay(i, true);
      else if (isOff)
        applyManualRelay(i, false);
      else if (isToggle)
        applyManualRelay(i, !relayState[i]);
    }
    publishAllRelayState();
    return;
  }

  // 2. Topic .../relay/1 .. /4 (kontrol relay per kanal)
  for (uint8_t i = 0; i < 4; i++) {
    char tCh[80];
    snprintf(tCh, sizeof(tCh), "%s/%d", TOPIC_RELAY_BASE, i + 1);
    if (strcmp(topic, tCh) == 0) {
      if (isOn)
        applyManualRelay(i, true);
      else if (isOff)
        applyManualRelay(i, false);
      else if (isToggle)
        applyManualRelay(i, !relayState[i]);
      else
        Serial.println("  [MQTT] Payload relay tidak dikenali");
      publishAllRelayState();
      return;
    }
  }

  // 3. Topic command lama/umum: topCmd
  // Relay legacy: "r1on" / "r1off" ... "r4off"
  char msgLower[32];
  for (unsigned int i = 0; i < n; i++)
    msgLower[i] = tolower((char)payload[i]);
  msgLower[n] = 0;

  if (msgLower[0] == 'r' && msgLower[1] >= '1' && msgLower[1] <= '4') {
    uint8_t ch = msgLower[1] - '1';
    if (!strcmp(msgLower + 2, "on"))
      applyManualRelay(ch, true);
    else if (!strcmp(msgLower + 2, "off"))
      applyManualRelay(ch, false);
    return;
  }

  if (!strcmp(msgLower, "auto")) { // kembalikan semua ke otomatis
    for (uint8_t i = 0; i < 4; i++)
      manualClear(i);
    publishEvent("manual_auto", "");
    return;
  }

  if (!strcmp(msgLower, "reset")) {
    for (uint8_t i = 0; i < 4; i++)
      relayLatch[i] = false;
    for (uint8_t i = 0; i < ALARM_COUNT; i++) {
      alarmSt[i].latched = false;
      alarmSt[i].active = false;
      alarmSt[i].confirm = 0;
    }
    guardTripped = false;
    ecRateStrikes = 0;
    levelCritLatch = false;
    for (uint8_t i = 0; i < 4; i++)
      manualClear(i);
    if (doseState == D_LOCKED) {
      doseState = D_IDLE;
      doseLockWhy = "";
    }
    publishEvent("reset", "manual");
  } else if (!strcmp(msgLower, "maint_on")) {
    maintenanceMode = true;
    publishEvent("maint", "on");
  } else if (!strcmp(msgLower, "maint_off")) {
    maintenanceMode = false;
    publishEvent("maint", "off");
  }
}

void loopWifi() { // non-blocking
  static uint32_t tLast = 0;
  static bool servicesStarted = false;

  if (WiFi.status() == WL_CONNECTED) {
    if (!servicesStarted) {
      servicesStarted = true;
      Serial.printf("\n[WIFI] Terhubung! IP: %s\n",
                    WiFi.localIP().toString().c_str());
      setupOta();
      setupTelnet();
    }
    return;
  }

  servicesStarted = false;
  if (millis() - tLast < WIFI_RETRY_MS)
    return;
  tLast = millis();
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loopMqtt() {
  if (WiFi.status() != WL_CONNECTED)
    return;
  if (mqtt.connected()) {
    mqtt.loop();
    return;
  }

  static uint32_t tLast = 0;
  if (millis() - tLast < MQTT_RETRY_MS)
    return;
  tLast = millis();

  bool ok;
  if (strlen(MQTT_USER))
    ok = mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS, topStatus, 1, true,
                      "{\"status\":\"offline\"}");
  else
    ok = mqtt.connect(MQTT_CLIENT_ID, topStatus, 1, true,
                      "{\"status\":\"offline\"}");
  if (ok) {
    mqtt.subscribe(topCmd);

    // Subscribe topic kendali relay per-channel dan all
    char subTopic[80];
    for (uint8_t i = 0; i < 4; i++) {
      snprintf(subTopic, sizeof(subTopic), "%s/%d", TOPIC_RELAY_BASE, i + 1);
      mqtt.subscribe(subTopic);
      Serial.printf("  [MQTT] Subscribe %s\n", subTopic);
    }
    snprintf(subTopic, sizeof(subTopic), "%s/all", TOPIC_RELAY_BASE);
    mqtt.subscribe(subTopic);
    Serial.printf("  [MQTT] Subscribe %s\n", subTopic);

    publishHeartbeat();
    publishAllRelayState();
    flushEvents(); // alarm tertunda dikirim lebih dulu
  }
}

// =====================================================================
//  9. SETUP / LOOP
// =====================================================================

void setup() {
  // --- kondisi aman: pastikan seluruh relay mati saat alat baru dinyalakan
  // (boot) ---
  for (uint8_t i = 0; i < 4; i++) {
    relayState[i] = false;
    manualOn[i] = false;
    manualHold[i] = false;
    relayLatch[i] = false;
    relayOnSince[i] = 0;
    applyPin(i, false);
    pinMode(relayPin[i], OUTPUT);
    applyPin(i, false);
  }

  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] HydroController");
  Serial.println("Ketik SCAN untuk cek slave ID 1..8 di bus RS485");

  if (RS485_DE_PIN >= 0) {
    pinMode(RS485_DE_PIN, OUTPUT);
    rs485Tx(false);
  }
  if (FLOAT_HIGH_PIN >= 0)
    pinMode(FLOAT_HIGH_PIN, FLOAT_ACTIVE_LOW ? INPUT_PULLUP : INPUT);

  RS485.begin(RS485_BAUD, RS485_CONFIG, RS485_RX_PIN, RS485_TX_PIN);

  snprintf(topStatus, sizeof(topStatus), "%s/status", MQTT_BASE);
  snprintf(topTele, sizeof(topTele), "%s/telemetry", MQTT_BASE);
  snprintf(topAlarm, sizeof(topAlarm), "%s/alarm", MQTT_BASE);
  snprintf(topEvent, sizeof(topEvent), "%s/event", MQTT_BASE);
  snprintf(topCmd, sizeof(topCmd), "%s/cmd", MQTT_BASE);

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);
  mqtt.setBufferSize(512);

  doseWindowStart = millis();

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wcfg = {WDT_TIMEOUT_S * 1000,
                                (1 << portNUM_PROCESSORS) - 1, true};
  esp_task_wdt_reconfigure(&wcfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  publishEvent("boot", "");
}

void loop() {
  esp_task_wdt_reset();

  // --- Polling Bergilir Berurutan (Round-Robin Terjadwal) ---
  // Terdapat 8 node pada 1 jalur RS485 bersama (7 Sensor + 1 Layar DWIN HMI).
  // Masing-masing node mendapat 1 slot waktu terisolasi sehingga TIDAK AKAN
  // PERNAH tabrakan.
  static uint32_t tPoll = 0;
  static uint8_t slotIdx = 0;
  const uint8_t TOTAL_BUS_SLOTS =
      SENSOR_COUNT + 1; // 7 sensor (0..6) + 1 DWIN HMI (7)
  const uint32_t slotTime =
      POLL_INTERVAL_MS / TOTAL_BUS_SLOTS; // 125 ms per slot

  if (millis() - tPoll >= slotTime) {
    if (slotIdx < SENSOR_COUNT) {
      // Slot 0 s/d 6: Tanya 1 sensor secara bergilir
      pollOne(slotIdx);
    } else {
      // Slot 7: Sapuan 7 sensor telah lengkap.
      // 1. Hitung data gabungan sensor (median, turunan, dsb)
      updateDerived();
      evaluateAlarms();
      controlDosing();
      controlClimate();
      updateDwinRegisters();

      // 2. Sinkronisasikan data terbaru dengan Layar DWIN HMI (Slave ID 8)
      syncDwinMaster();
    }

    slotIdx = (slotIdx + 1) % TOTAL_BUS_SLOTS;
    tPoll = millis(); // Reset penghitung waktu slot setelah transaksi selesai
    delay(MODBUS_GAP_MS); // Jeda diam wajib (>3.5 char) agar transceiver RS485
                          // tenang
  }

  // --- scanner serial Modbus ---
  handleSerialCommands();

  // --- override manual + keselamatan: SELALU, tanpa syarat ---
  applyManual();
  relayGuard();

  // --- telemetri ---
  static uint32_t tTele = 0, tHb = 0, tSerial = 0;
  if (millis() - tTele >= TELEMETRY_MS ||
      (movedEnough() && millis() - tTele > 5000)) {
    tTele = millis();
    publishTelemetry();
  }
  if (millis() - tHb >= HEARTBEAT_MS) {
    tHb = millis();
    publishHeartbeat();
  }
  if (millis() - tSerial >= 5000UL) {
    tSerial = millis();
    printSensorSerial();
  }

  loopWifi();
  loopMqtt();
  loopOta();
  loopTelnet();
}
