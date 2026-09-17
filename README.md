# SmartHidroponik - HydroController Firmware (ESP32)

Firmware kontroler hidroponik otomatis berbasis ESP32 dengan komunikasi industri RS485 Modbus RTU dan HMI DWIN.

## 📌 Ringkasan Sistem
Sistem ini mengendalikan sistem hidroponik terpadu secara cerdas dan andal:
- **Arsitektur Bus Tunggal RS485 Modbus RTU**: ESP32 bertindak sebagai Master tunggal yang melakukan polling sensor dan HMI secara sekuensial tanpa tabrakan transmisi (*collision-free*).
- **Sensor Modbus Terintegrasi**:
  - Sensor Suhu & Kelembapan Udara (Asair)
  - Sensor Intensitas Cahaya (Lux)
  - Sensor Suhu/Kelembapan Tanah / Air (CWT03S)
  - Sensor EC / TDS (ECTDS10-ISO)
  - Sensor pH Air
  - Sensor Tambahan (CWTHXXS)
  - Sensor Level Ketinggian Tandon Laser (WT53R-485)
- **HMI DWIN Touchscreen (Modbus Slave ID 8)**: Komunikasi ultra-cepat dengan Modbus Function Code 0x10 (*Write Multiple Registers*) untuk pembaruan instan sensor & status relay.
- **Pengendalian Aktuator**:
  - Pompa Nutrisi (dengan State Machine Dosing: Idle, Dosing, Mixing, Locked)
  - Pompa Misting
  - Exhaust Fan
  - Pompa Sirkulasi Utama
- **Keandalan & Filter Data**: CRC check, *stuck sensor detection*, filter batas (*valid min/max*), *rate limiting*, dan *median filter* 5-titik.

## 📂 Struktur Repositori
```text
├── HydroController/
│   ├── HydroController.ino    # Logika utama firmware ESP32
│   ├── config.h               # Konfigurasi pin, parameter dosing, batas alarm
│   └── types.h                # Definisi struct, enum, dan tipe data
├── dokumentasi_sistem.md      # Panduan teknis arsitektur & alur logika
├── keandalan_komunikasi_modbus.md # Analisis keandalan komunikasi Modbus
├── walkthrough.md             # Catatan optimalisasi & implementasi DWIN
└── README.md
```

## 🛠️ Persyaratan & Instalasi
1. **Arduino IDE / PlatformIO** dengan board package ESP32 (ESP32-S3 atau board yang kompatibel).
2. Library:
   - Arduino core for ESP32
3. Buka folder `HydroController/` di Arduino IDE, pilih board dan port COM yang sesuai, lalu lakukan compile dan upload.

## 📖 Dokumentasi Lengkap
Silakan baca file dokumentasi berikut untuk detail teknis:
- [Dokumentasi Sistem](dokumentasi_sistem.md)
- [Keandalan Komunikasi Modbus](keandalan_komunikasi_modbus.md)
