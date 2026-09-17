// =====================================================================
//  types.h — Definisi tipe & prototipe.
//
//  MENGAPA BERKAS INI ADA:
//  Arduino IDE menyisipkan prototipe fungsi secara otomatis di bagian
//  ATAS berkas .ino, sebelum definisi struct yang ada di dalamnya.
//  Akibatnya prototipe seperti
//      bool medianOfAir(Median *f, bool *ok, float &out);
//  muncul saat `Median` belum dikenal, gagal diurai, dan compiler
//  melaporkan "'medianOfAir' cannot be used as a function".
//
//  Karena penyisipan itu dilakukan SETELAH seluruh baris #include,
//  memindahkan tipe ke header menyelesaikan masalah secara permanen.
//  Jangan memindahkan struct di bawah ini kembali ke .ino.
// =====================================================================
#ifndef TYPES_H
#define TYPES_H

#include "config.h"
#include <Arduino.h>
#include <math.h>

// ------------------------------------------------------ filter median
struct Median {
  float buf[MEDIAN_N];
  uint8_t n = 0, idx = 0;
  bool ready = false;
  float value = NAN;

  void push(float v) {
    buf[idx] = v;
    idx = (idx + 1) % MEDIAN_N;
    if (n < MEDIAN_N)
      n++;
    float t[MEDIAN_N];
    for (uint8_t i = 0; i < n; i++)
      t[i] = buf[i];
    for (uint8_t i = 1; i < n; i++) { // insertion sort
      float k = t[i];
      int8_t j = i - 1;
      while (j >= 0 && t[j] > k) {
        t[j + 1] = t[j];
        j--;
      }
      t[j + 1] = k;
    }
    value = t[n / 2];
    ready = (n >= 3);
  }
  void reset() {
    n = 0;
    idx = 0;
    ready = false;
    value = NAN;
  }
};

// ------------------------------------------------------- node sensor
struct SensorDef {
  uint8_t id;
  uint8_t fc;
  uint16_t addr;
  uint8_t qty;
};

struct NodeState {
  uint8_t failStreak = 0;
  uint16_t stuckCount = 0;
  uint16_t lastRaw[8];
  bool online = false;
};

// ------------------------------------------------------------- alarm
enum AlarmCode {
  C01A,
  C01B,
  C02,
  C03,
  C04,
  C05,
  C06,
  C07,
  C08,
  C09,
  C10,
  W01,
  W02,
  W02B,
  W03,
  W04,
  W06,
  W07,
  W08,
  W09,
  W10,
  W11,
  W12,
  W13,
  W15,
  W16,
  W20,
  ALARM_COUNT
};

struct AlarmDef {
  const char *code;
  bool critical;
};

struct AlarmState {
  bool active = false;
  bool latched = false;
  uint8_t confirm = 0;
  uint32_t lastPub = 0;
  uint32_t since = 0;
};

// ------------------------------------------------- mesin keadaan dosis
enum DoseState { D_IDLE, D_DOSING, D_MIXING, D_LOCKED };

// ------------------------------------------- kejadian tertunda (offline)
struct Event {
  char kind[16];
  char detail[24];
  uint32_t ts;
};

// ========================== PROTOTIPE ================================
// Dideklarasikan eksplisit agar penyisipan otomatis Arduino tidak
// pernah menjadi satu-satunya sumber deklarasi.

uint16_t modbusCRC(const uint8_t *buf, uint8_t len);
void rs485Tx(bool on);
bool modbusRead(uint8_t id, uint8_t fc, uint16_t addr, uint8_t qty,
                uint16_t *out);
bool modbusWrite(uint8_t id, uint16_t addr, uint16_t val);
bool modbusWriteMultiple(uint8_t id, uint16_t addr, uint8_t qty,
                         const uint16_t *vals);
float s16(uint16_t v);

void decodeNode(uint8_t k, uint16_t *r);
bool medianOfAir(Median *f, bool *ok, float &out);
void pollOne(uint8_t k);
void updateDerived();
void syncDwinMaster();

void applyPin(int ch, bool on);
void setRelay(uint8_t ch, bool on);
void forceOff(uint8_t ch);
void relayGuard();
bool floatHigh();
void publishRelayState(uint8_t idx);
void publishAllRelayState();

void publishEvent(const char *kind, const char *detail);
void publishAlarm(uint8_t i, bool on);
void evalAlarm(uint8_t i, bool cond);
bool isActive(uint8_t i);
bool anyNodeOffline();
bool anyNodeStuck();
bool nodeStuck(uint8_t idx);
bool nodeBad(uint8_t idx);
uint8_t activeAlarmList(char *dst, size_t n);
void evaluateAlarms();

const char *doseStateName();
void doseLock(const char *why);
bool manualActive(uint8_t ch);
void manualClear(uint8_t ch);
bool manualAllowed(uint8_t ch, bool on);
void applyManual();
void controlDosing();
void controlClimate();

void evPush(const char *kind, const char *detail);
void flushEvents();
bool movedEnough();
void fmt(char *dst, size_t n, float v, uint8_t dec);
void publishTelemetry();
void publishHeartbeat();
void onMqtt(char *topic, byte *payload, unsigned int len);
void loopWifi();
void loopMqtt();
void getTimestamp(char *dst, size_t n);

#endif // TYPES_H
