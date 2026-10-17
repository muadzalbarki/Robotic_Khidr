# Setup Skills & OpenCode Plugin

## 1. Konfigurasi OpenCode Plugin (Superpowers)
Tambahkan konfigurasi berikut ke file `opencode.json` di root direktori proyek Anda:

```json
{
  "plugins": ["superpowers@git+https://github.com/obra/superpowers.git"]
}
```

## 2. Struktur Direktori Lokal
Folder `.opencode/skills/` dan `.opencode/plugins/` telah dibuat di root proyek untuk menyimpan konfigurasi khusus agent dan custom skill.

## 3. Metodologi TDD (Test-Driven Development)
- **ESP32 (C++):** Gunakan framework testing seperti Unity atau AUnit, tulis test terlebih dahulu sebelum implementasi driver/motor/servo.
- **Raspberry Pi 5 (Python):** Gunakan `unittest` atau `pytest` untuk sensor fusion (LiDAR + Vision) dan state machine mission.
