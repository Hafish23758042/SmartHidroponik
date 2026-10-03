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
2. **Library yang Dibutuhkan** (dapat diinstall via Library Manager):
   - **WiFiManager** (oleh *tzapu*, versi 2.0.x ke atas)
   - **PubSubClient** (oleh *Nick O'Leary*)
   - ArduinoOTA & WiFi (bawaan ESP32 core)
3. Buka folder `HydroController/` di Arduino IDE, pilih board dan port COM yang sesuai, lalu lakukan compile dan upload.

## 📶 Konfigurasi WiFi (WiFiManager Captive Portal)
1. Saat pertama kali dinyalakan atau bila WiFi tidak terjangkau, ESP32 akan membuat Access Point sendiri bernama **`HydroController-AP`** (tanpa password secara default).
2. Hubungkan smartphone / laptop ke WiFi **`HydroController-AP`**.
3. Portal captive web akan otomatis terbuka (atau buka browser ke alamat IP `192.168.4.1`).
4. Pilih **Configure WiFi**, pilih SSID jaringan lokal Anda, masukkan password, lalu simpan (*Save*).
5. ESP32 akan tersambung ke jaringan lokal Anda dan menyimpan kredensial ke memori internal (NVS).
6. **Keamanan & Keandalan**:
   - Jika portal tidak disentuh selama 180 detik (*timeout*), sistem akan otomatis melanjutkan eksekusi agar kontrol nutrisi dan pembacaan sensor tetap berjalan normal (*offline mode*).
   - Untuk mereset kredensial WiFi, kirim perintah `RESETWIFI` melalui Serial Monitor (115200 baud) atau publish topic MQTT `hydroponik/unit01/cmd` dengan pesan `resetwifi`.

## 📖 Dokumentasi Lengkap
Silakan baca file dokumentasi berikut untuk detail teknis:
- [Dokumentasi Sistem](dokumentasi_sistem.md)
- [Keandalan Komunikasi Modbus](keandalan_komunikasi_modbus.md)
