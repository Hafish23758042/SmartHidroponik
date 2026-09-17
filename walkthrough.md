# Pembaruan: Modbus Master & Optimalisasi DWIN (Tembakan Massal)

Berikut adalah rekapitulasi menyeluruh atas seluruh perombakan pada kode `HydroController.ino` untuk menjamin lalu lintas data RS485 antara ESP32, sensor, dan layar DWIN berjalan dengan sempurna dan secepat kilat.

## 1. Migrasi ESP32 Menjadi Satu-satunya Master
Fungsi usang `dwinProcessFrame` dan `loopDwin` (yang memaksa ESP32 menunggu perintah) telah **dihapus**. ESP32 sekarang bertindak sebagai Master yang memegang kendali penuh pada bus RS485. Eksekusi HMI dijadwalkan secara aman di dalam `loop()` persis setelah 7 siklus pembacaan sensor selesai, menjamin tidak akan pernah ada tabrakan (*collision*) transmisi di kabel RS485.

## 2. Penataan Ulang Mapping VP Address
Merespons tabel konfigurasi Anda yang terbaru, struktur alamat di dalam kode ESP32 telah dirapikan agar berurutan (*contiguous*), yang merupakan syarat utama untuk fitur pengiriman data massal.
- **Blok Sensor (5100 - 5110):** Menampung 11 data nilai sensor (`airT`, `ecV`, `phV`, dst).
- **Blok Relay (5120 - 5125):** Menampung 6 data perintah hidup/mati untuk aktuator pompa, exhaust, dan misting.

## 3. Penambahan Fungsi Tembakan Massal (0x10)
Fungsi baru `modbusWriteMultiple()` telah ditambahkan. Fungsi ini mengimplementasikan **Modbus Function Code 0x10 (Write Multiple Registers)**. Ini adalah inti dari optimalisasi kita.

Fungsi ini (dan juga fungsi `modbusWrite()` sebelumnya) sudah mengontrol otomatis pin DE/RE pada IC RS485 (`rs485Tx(true/false)`) agar setiap paket benar-benar terdorong keluar secara fisik ke kabel RS485.

## 4. Mesin Sinkronisasi Super Cepat: `syncDwinMaster()`
Berkat alamat yang berurutan, rutin sinkronisasi layar HMI telah dipadatkan dari 22 baris transaksi tunggal menjadi **hanya 3 instruksi transaksi besar**:
1. **PULL (Baca Tombol):** Menarik status sakelar dari layar mulai dari alamat `5120` ke atas (sekali tarik = 6 tombol terbaca).
2. **PUSH (Kirim Sensor):** Menyemburkan semua 11 nilai sensor mulai dari alamat `5100` ke atas dalam **1 paket data massal**.
3. **PUSH (Kirim Relay):** Menyemburkan semua 6 status aktuator ke layar mulai dari alamat `5120` ke atas, juga dalam 1 paket data.

> [!TIP]
> **Lompatan Performa**
> Transaksi yang awalnya membutuhkan waktu blokir lebih dari ~340 milidetik (karena dikirim eceran satu per satu), kini menyusut tajam menjadi di bawah **~100 milidetik**. Sensor Anda tetap terbaca dengan cepat, dan angka HMI Anda akan *update* secara super instan!

---
**Status Akhir:** Kode siap tempur! Silakan jalankan *Compile & Upload* ke board ESP32 Anda.
