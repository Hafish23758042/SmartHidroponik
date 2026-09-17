// =====================================================================
//  config.h — Seluruh parameter yang boleh disetel ada di file ini.
//  Jangan menaruh angka ajaib di dalam logika kendali.
// =====================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ---------------------------------------------------------------- RS485
// Pin terpakai lain: 21/47/14/38 relay, 19/20 USB-JTAG bawaan S3.
#define RS485_RX_PIN 16
#define RS485_TX_PIN 15
#define RS485_DE_PIN -1 // modul auto-direction: arah tak dikendalikan MCU
#define RS485_BAUD 9600
#define RS485_CONFIG SERIAL_8N1

// DWIN HMI dan semua sensor berada di BUS RS485 yang SAMA.
// Jadi pin TX/RX-nya harus dipakai bersama 1 pair (A/B) yang sama.
// ESP32 ini bertindak sebagai SATU-SATUNYA MASTER di jaringan ini,
// secara aktif me-request sensor dan membaca/menulis ke HMI (Slave ID 8).
#define DWIN_SLAVE_ID 8

// ------------------------------------ Mode Hemat Energi Layar DWIN (Standby)
// Layar DMG10600C101_15WTR (T5L DGUS II) mendukung auto sleep / backlight off.
// Saat standby tercapai, backlight mati (0%). Saat layar disentuh, hardware
// otomatis menyalakan backlight kembali ke nilai ON (Touch-to-Wake aman).
#define DWIN_STANDBY_ENABLED true
#define DWIN_BRIGHTNESS_ON 100 // Kecerahan saat aktif (0-100%)
#define DWIN_BRIGHTNESS_STANDBY                                                \
  0 // Kecerahan saat standby (0% = mati total / hemat energi)
#define DWIN_STANDBY_TIMEOUT_S                                                 \
  60 // Waktu tunggu mati otomatis tanpa sentuhan (detik)

// Slot per node: 8 node (7 sensor + 1 layar DWIN HMI).
// Total 1000 ms / 8 node = 125 ms per slot transaksi.
// Timeout 150 ms cukup untuk 9600 baud tanpa memblokir bus bila ada node yang
// mati.
#define MODBUS_TIMEOUT_MS 150
#define MODBUS_GAP_MS                                                          \
  12 // jeda diam antar transaksi (>3.5 char @9600 baud, standar t3.5 RS485)

// ---------------------------------------------------------------- Relay
const int relayPin[4] = {21, 47, 14, 38}; // 1..4
// Sebagian besar modul relay adalah ACTIVE LOW (LOW=ON, HIGH=OFF) atau ACTIVE
// HIGH (HIGH=ON, LOW=OFF). Diubah ke false agar saat standby/boot pin bernilai
// LOW (Relay OFF untuk modul Active HIGH).
const bool RELAY_ACTIVE_LOW = false; // false = Active HIGH (LOW=OFF, HIGH=ON)

// Kanal 4 sengaja dinonaktifkan pada tahap ini.
const bool relayEnabled[4] = {true, true, true, true}; // <-- Pin 38 kini AKTIF

const char *const relayName[4] = {"pompa_nutrisi", "misting", "exhaust_fan",
                                  "pompa_utama"};

enum RelayCh { R_PUMP = 0, R_MIST = 1, R_FAN = 2, R_PUMP_UTAMA = 3 };

// Batas durasi aktif MUTLAK. 0 = tanpa batas (jangan dipakai untuk air).
const uint32_t RELAY_MAX_ON_MS[4] = {
    5000UL,   // pompa nutrisi  : 5 detik
    30000UL,  // misting        : 30 detik
    600000UL, // exhaust fan    : 10 menit
    0UL       // pompa utama    : tanpa batas (atau sesuaikan bila perlu)
};

// ------------------------------------------------------- Sakelar pelampung
// TIDAK TERPASANG. Nilai -1 menonaktifkan pembacaan sepenuhnya:
// floatHigh() selalu mengembalikan false dan alarm C08 tidak pernah naik.
//
// Bila nanti dipasang, isi dengan GPIO yang benar-benar bebas — JANGAN 15
// (TX RS485), 16 (RX RS485), atau 14/21/38/47 (relay). Pin yang dibiarkan
// mengambang akan terbaca LOW dan mengunci pompa tanpa sebab.
#define FLOAT_HIGH_PIN -1     // -1 = tidak terpasang
#define FLOAT_ACTIVE_LOW true // true = INPUT_PULLUP, LOW berarti aktif

// -------------------------------------------------- Sensor slave ID
// Ini adalah alamat MODBUS sensor yang dipanggil oleh ESP32 master.
// DWIN HMI adalah entitas terpisah dan sudah diatur dengan ID 8 di bagian atas
// file.
#define ID_ASAIR 1
#define ID_LUX 2
#define ID_CWT03S 3
#define ID_ECTDS 4
#define ID_PH 5
#define ID_CWTHXXS 6
#define ID_LEVEL 7
#define SENSOR_COUNT 7

// ------------------------------------------- Level tandon (WT53R-485)
// Sensor laser dipasang di ATAS tandon menghadap ke bawah, sehingga
// jarak MENGECIL saat tandon terisi. Ukur kedua nilai berikut secara
// langsung di lapangan — jangan dihitung dari gambar teknik.
#define LEVEL_EMPTY_MM 800.0f // jarak sensor -> dasar tandon (0%)
#define LEVEL_FULL_MM 150.0f  // jarak sensor -> permukaan penuh (100%)

#define DIST_VALID_MIN 40.0f   // spesifikasi WT53R: 4 cm
#define DIST_VALID_MAX 4000.0f // spesifikasi WT53R: 400 cm

#define LEVEL_WARN_PCT 30.0f   // W20 peringatan
#define LEVEL_CRIT_PCT 10.0f   // C10 kritis: kunci pompa
#define LEVEL_RESUME_PCT 20.0f // histeresis pemulihan

// ------------------------------------------------- Titik operasi dosing
// Faktor TDS pada ECTDS10-ISO = 0.50  ->  ppm = uS/cm * 0.50
// PERIKSA skala TDS pen Anda (0.5 vs 0.7) sebelum produksi.
#define TDS_FACTOR 0.50f

#define EC_DOSE_START 1600.0f // < ini  -> mulai dosis  (800 ppm)
#define EC_DOSE_STOP 1900.0f  // >= ini -> henti dosis  (950 ppm)
#define EC_TARGET_MAX 2000.0f // sasaran maksimum      (1000 ppm)

#define DOSE_MS 2000UL       // 1 dosis = 2 detik
#define MIX_WAIT_MS 600000UL // jeda pencampuran = 10 menit
#define DOSE_MAX_PER_DAY 12
#define DOSE_WINDOW_MS 86400000UL // jendela 24 jam

// ------------------------------------------------ Validasi sensor (EC)
#define EC_VALID_MIN 50.0f   // < ini: elektroda kering / kabel putus
#define EC_VALID_MAX 5000.0f // > ini: di luar rentang budidaya
#define EC_RATE_MAX 300.0f   // lompatan tak wajar antar poll
#define EC_RATE_STRIKES 3    // berturut-turut -> kunci

#define PH_VALID_MIN 3.0f
#define PH_VALID_MAX 11.0f
#define WTEMP_VALID_MIN -5.0f
#define WTEMP_VALID_MAX 60.0f
#define ATEMP_VALID_MIN -20.0f
#define ATEMP_VALID_MAX 70.0f
#define RH_VALID_MIN 0.0f
#define RH_VALID_MAX 100.0f

// Pembacaan identik berturut-turut sebelum sensor dianggap beku.
// Pada 1 Hz, 900 = 15 menit. Nilai 60 (1 menit) TERLALU AGRESIF:
// elektroda EC di larutan diam wajar membaca angka sama persis
// selama bermenit-menit, dan itu memicu alarm palsu.
#define STUCK_LIMIT 900
#define COMM_FAIL_LIMIT 5 // kegagalan berturut-turut -> C06

// ----------------------------------------------- Ambang KRITIS (latch)
#define EC_CRIT_LOW 50.0f     // C01a  (< 25 ppm)
#define EC_CRIT_HIGH 2800.0f  // C01b  (> 1400 ppm)
#define WTEMP_CRIT_HIGH 33.0f // C05
#define PH_CRIT_LOW 4.5f      // C09
#define PH_CRIT_HIGH 8.0f

// ------------------------------------------- Ambang PERINGATAN + histeresis
#define EC_WARN_LOW 1500.0f    // W01 (750 ppm)
#define EC_WARN_HIGH 2000.0f   // W02 (1000 ppm)
#define EC_WARN_HIGH2 2300.0f  // W02b (1150 ppm)
#define PH_WARN_LOW 5.5f       // W03
#define PH_WARN_HIGH 6.8f      // W04
#define WTEMP_WARN_HIGH 30.0f  // W06
#define WTEMP_WARN_LOW 18.0f   // W07
#define WTEMP_DIVERGE 1.5f     // W08 (deteksi sirkulasi mati)
#define ATEMP_WARN_HIGH 32.0f  // W09
#define ATEMP_WARN_HIGH2 35.0f // W10
#define RH_WARN_HIGH 85.0f     // W11
#define RH_WARN_LOW 50.0f      // W12
#define AIR_DIVERGE_T 3.0f     // W13
#define AIR_DIVERGE_RH 10.0f   // W13
#define RS485_ERR_PCT 5.0f     // W15

// ------------------------------------------------------ Kendali iklim
#define CLIMATE_T_ON 32.0f    // aktif  di atas ini
#define CLIMATE_T_OFF 30.0f   // mati   di bawah ini (histeresis 2 C)
#define MIST_RH_CEILING 75.0f // RH > ini -> misting dilarang
#define FAN_RH_ON 85.0f
#define FAN_RH_OFF 78.0f

#define MIST_BURST_MS 20000UL   // 20 detik per siklus
#define MIST_PERIOD_MS 600000UL // maksimum 1 siklus / 10 menit

// -------------------------------------------------------- Penjadwalan
#define POLL_INTERVAL_MS 1000UL // satu sapuan penuh 7 sensor
#define MEDIAN_N 5
#define ALARM_CONFIRM_CRIT 3
#define ALARM_CONFIRM_WARN 5
#define ALARM_REPUBLISH_MS 1800000UL // 30 menit

// ------------------------------------------------ Kendali manual (MQTT)
// Perintah manual KEDALUWARSA sendiri. Tanpa ini, satu perintah "ON"
// yang terlupakan akan menyalakan aktuator sampai ada yang menyadarinya
// — persis mode kegagalan yang ingin dicegah seluruh rancangan ini.
#define MANUAL_TIMEOUT_MS 600000UL // 10 menit, lalu kembali otomatis

// -------------------------------------------------------- Watchdog
#define WDT_TIMEOUT_S 30

// -------------------------------------------------------- NTP
// Sinkronisasi jam dari internet agar timestamp MQTT akurat.
// ESP32 menggunakan SNTP bawaan (configTime), tidak perlu library tambahan.
#define NTP_SERVER1 "pool.ntp.org"
#define NTP_SERVER2 "id.pool.ntp.org" // server NTP Indonesia
#define NTP_GMT_OFFSET 25200          // WIB = UTC+7 = 7*3600
#define NTP_DAYLIGHT 0                // Indonesia tidak pakai DST

// -------------------------------------------------------- Jaringan & Layanan
#define WIFI_SSID "IET"
#define WIFI_PASS "meeting!!"
#define WIFI_RETRY_MS 30000UL

// OTA (Over-The-Air Update)
#define OTA_HOSTNAME "HydroController-ESP32"
#define OTA_PASSWORD "admin123" // password update OTA
#define OTA_PORT 3232

// Telnet Serial Monitor
#define TELNET_PORT 23

#define MQTT_HOST "sdp.polinela.ac.id"
#define MQTT_PORT 1883
#define MQTT_USER "septa"
#define MQTT_PASS "123321"
#define MQTT_CLIENT_ID "hydroponik"
#define MQTT_BASE "hydroponik/unit01"
#define TOPIC_RELAY_BASE "polinela/lab/relay"
#define MQTT_RETRY_MS 5000UL
#define TELEMETRY_MS 60000UL // telemetri berkala
#define HEARTBEAT_MS 60000UL

// Delta pemicu pengiriman di luar jadwal
#define DELTA_TDS 25.0f
#define DELTA_PH 0.1f
#define DELTA_WTEMP 0.3f
#define DELTA_ATEMP 0.5f
#define DELTA_RH 3.0f

#define EVENT_BUF_SIZE 64 // ring buffer kejadian saat offline

#endif // CONFIG_H
