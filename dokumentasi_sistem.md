# Dokumentasi Lengkap Firmware HydroController ESP32

Dokumen ini memuat penjelasan menyeluruh mengenai arsitektur, alur logika, dan cara kerja dari sistem _firmware_ pengontrol hidroponik berbasis ESP32. Sistem ini dirancang dengan mengutamakan **keandalan industri**, toleransi terhadap kegagalan perangkat keras, dan keamanan data.

---

## 1. Struktur File Proyek
Kode sumber dibagi menjadi tiga *file* utama agar mudah dikelola dan dimodifikasi tanpa merusak logika inti:

*   **`config.h`**
    Pusat pengaturan (Dashboard Parameter). Semua konfigurasi sistem keras (Pin, Baudrate, Slave ID), parameter dosing (batas EC, volume), batas alarm (Kritis/Peringatan), dan konfigurasi WiFi/MQTT ada di sini. **Aturan utama:** Tidak boleh ada "angka ajaib" (angka yang ditanam langsung) di dalam kode logika utama, semuanya ditarik dari file ini.
*   **`types.h`**
    Definisi tipe data (*struct* dan *enum*). Berisi kerangka untuk objek Sensor, Status Node, Mesin Status (Dosing), Alarm, dan Buffer Event. Ini membuat kode lebih terstruktur.
*   **`HydroController.ino`**
    Jantung penggerak (Logika Eksekusi). Memuat seluruh algoritma pembacaan Modbus, filter median data, logika kontrol iklim, mesin *dosing* nutrisi, sinkronisasi layar DWIN HMI, dan komunikasi MQTT.

---

## 2. Arsitektur Komunikasi Modbus RTU Tunggal
Sistem ini menggunakan **Topologi Bus Tunggal (1 Kabel RS485)**.
*   **Peran ESP32:** Sebagai **SATU-SATUNYA MASTER** di dalam jaringan.
*   **Peran Sensor (ID 1-7):** Sebagai *Slave* pasif yang hanya akan bicara jika ditanya.
*   **Peran Layar DWIN (ID 8):** Juga disetel sebagai *Slave* pasif.

**Mencegah Tabrakan Data (Collision Avoidance):**
ESP32 menggunakan metode *polling* (tanya-jawab bergiliran).
1. Tanya Sensor 1 -> Tunggu Jawaban
2. Tanya Sensor 2 -> Tunggu Jawaban
3. ... (sampai Sensor 7)
4. *Sync* Layar DWIN -> Tarik Status Tombol -> Dorong Nilai Sensor Baru
Dengan pola berurutan dan disiplin ketat (`rs485Tx()` dan `RS485.flush()`), data tidak akan pernah bertabrakan di dalam kabel.

---

## 3. Integrasi HMI DWIN Layar Sentuh (Tembakan Massal)
Agar *update* angka di layar tidak memperlambat laju pembacaan sensor, kita menggunakan fitur **Modbus Multiple Registers (Tembakan Massal)**.
*   **Alamat Berurutan (Contiguous):** Variabel di layar diatur berurutan (5100-5110 untuk sensor, 5120-5123 untuk relay/tombol).
*   **Fungsi `modbusWriteMultiple`:** Mengirim belasan angka (suhu, kelembapan, EC, pH, dll) dalam **1 paket besar (Fungsi 0x10)** sekaligus, memangkas waktu proses dari 340ms menjadi di bawah 100ms.
*   **Sinkronisasi 2-Arah:** Saat Anda menekan tombol di layar DWIN, statusnya dibaca oleh ESP32 melalui paket massal (Fungsi 0x03), kemudian ESP32 mencocokkannya dan menyalakan/mematikan sakelar (Relay) fisik yang sesuai (Pompa Nutrisi, Exhaust, Misting, atau Pompa Utama).

---

## 4. Keandalan Sensor (Filter & Keamanan Data)
Nilai dari sensor **tidak langsung dipercaya**. Sebelum dipakai untuk menggerakkan pompa, data disaring lewat beberapa lapisan:
*   **Validasi Modbus (CRC):** Cek keutuhan paket data elektronis. Jika cacat, paket dibuang.
*   **Deteksi Perangkat Beku (Stuck):** Jika sensor membalas dengan angka yang persis sama selama bermenit-menit (sensor *hang*), data ditolak.
*   **Filter Batas Mutlak (Valid_Min & Valid_Max):** EC bernilai `30000` (korsleting) atau `-50` akan langsung dibuang.
*   **Laju Perubahan (Rate Limit):** Jika EC mendadak melonjak naik 500 poin dalam sedetik, dianggap ada gelembung udara di alat, data ditahan (*strike*).
*   **Filter Median 5-Titik:** 5 sampel terakhir diurutkan secara matematis, dan diambil nilai tengahnya (bukan rata-ratanya) untuk membuang _noise_ (puncak/lembah yang tidak wajar akibat gangguan sinyal sesaat).

---

## 5. Mesin Kontrol Nutrisi Otomatis (Dosing)
Logika *dosing* tidak sekadar ON/OFF jika EC kurang. Sistem memakai **State Machine** (Mesin Status) untuk mencegah larutan menjadi overdosis:
1.  **`D_IDLE`:** Menunggu santai. Jika EC tervalidasi benar-benar turun di bawah target, pindah ke `D_DOSING`.
2.  **`D_DOSING`:** Menyalakan Pompa Nutrisi selama hitungan detik yang diizinkan (`DOSE_MS`).
3.  **`D_MIXING`:** Pompa dimatikan. Mesin **terkunci** (menunggu larutan diaduk secara merata di dalam tandon) selama `MIX_WAIT_MS` (misal 10 menit). Hal ini menjamin sensor membaca EC asli larutan yang sudah rata sebelum memutuskan butuh tambahan dosis atau tidak.
4.  **`D_LOCKED`:** Pompa dikunci mati secara total bila ada alarm kritis (sensor putus, pelampung air kosong, atau dosis harian berlebihan).

---

## 6. Mesin Kontrol Iklim (Climate Control)
Mengendalikan Kipas Exhaust dan Mesin Kabut (Misting) untuk menjaga area tumbuh-kembang tanaman.
*   **Exhaust Fan:** Aktif ketika kelembapan (RH) melampaui batas (*FAN_RH_ON*). Fan dijamin akan mati jika misting sedang berjalan (agar kabut tidak langsung tersedot terbuang keluar).
*   **Misting (Kabut):** Dipicu oleh sensor suhu ruangan. Hanya boleh menyala apabila kelembapan (RH) udara belum menyentuh batas embun (*MIST_RH_CEILING*), agar ruangan tidak menjadi kebanjiran air.

---

## 7. Sistem Peringatan (Alarm Matrix)
Setiap anomali diawasi oleh mesin konfirmasi terpusat. Kode peringatan dibedakan menjadi dua kelas:
*   **Warning (W01-W20):** Kesalahan minor atau mendekati batas toleransi (EC agak rendah, suhu agak tinggi). Alarm tercatat dan dikirim, tapi pompa tetap berjalan.
*   **Critical (C01-C10):** Bahaya fatal. Misalnya: Air nyaris habis (C10), kabel EC putus (C06/C07), EC *overdose* meledak (C01B). Ini memicu status *Latch* (Terkunci). Sistem akan memutus paksa aliran listrik ke pompa dan tidak akan menyala lagi hingga pengguna me-resetnya secara manual via tombol atau layar.

---

## 8. Pemantauan Jarak Jauh (MQTT & Telemetri)
ESP32 terkoneksi secara nirkabel (WiFi) untuk bertukar pesan dengan peladen (Server) pengelola (*broker*).
*   **Telemetry:** Mengirim denyut data angka (suhu, EC, pH) setiap 1 menit, ATAU lebih cepat jika terdeteksi fluktuasi angka yang sangat signifikan.
*   **Event & Alarm Log:** Setiap kali relay menyala, mesin dosing bergeser status, atau ada alarm pecah, ESP32 mengirim log kejadian (*event*). Jika internet putus, log disimpan di antrean memori internal (RAM) dan ditembakkan massal begitu internet menyambung kembali.
*   **Command Listener:** Menerima kendali sakelar dan *reset* jarak jauh. Jika manusia menyalakan pompa secara jarak jauh (*Manual Mode*), *timer* keselamatan darurat 10 menit akan diaktifkan untuk mencegah kelalaian mematikan pompa.

Sistem yang tangguh ini menjaga stabilitas biologis air, melindungi komponen elektronik, dan mendokumentasikan semua perilaku _hardware_ 24 jam penuh!
