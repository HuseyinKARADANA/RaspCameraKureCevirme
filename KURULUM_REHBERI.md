# KameraKüre - Raspberry Pi 5 Mavi Renk Takip ve 2 DC Motor Kontrolü

Bu proje, kamera görüntüsündeki **mavi renkli** bir nesneyi tespit eder, ekranın merkezinde belirlenen tolerans bölgesinde (ölü bölge) tutmak için **2 adet DC motoru** otomatik olarak sürer.

---

## 1. Donanım ve Pin Bağlantı Şeması

Raspberry Pi 5'in GPIO pinleri L298N (veya L293D) çift motor sürücüsüne bağlanır.

### L298N ile Raspberry Pi 5 Bağlantısı

| L298N Motor Sürücü Pini | Raspberry Pi 5 Pini | Açıklama |
| :--- | :--- | :--- |
| **IN1** | **GPIO 17** (Pin 11) | Motor 1 Yön Kontrol A |
| **IN2** | **GPIO 27** (Pin 13) | Motor 1 Yön Kontrol B |
| **IN3** | **GPIO 23** (Pin 16) | Motor 2 Yön Kontrol A |
| **IN4** | **GPIO 24** (Pin 18) | Motor 2 Yön Kontrol B |
| **GND** | **GND** (Örn: Pin 6, 9 veya 14) | **Çok Önemli: Pi ile ortak toprak (GND) hattı olmalıdır.** |
| **12V / Power** | Harici Pil / Güç Kaynağı | DC motorlar için güç (7V - 12V pil) |

> ⚠️ **UYARI**: DC motorları asla doğrudan Raspberry Pi'nin 5V pininden beslemeyin; Raspberry Pi 5 aşırı akımdan dolayı kapanabilir veya zarar görebilir. Motor sürücüsüne harici bir pil/adaptör bağlayın ve pilin eksi (-) kutbunu Raspberry Pi'nin GND pinine bağlayarak şaseleri ortaklayın.

---

## 2. Raspberry Pi 5 Kurulum Adımları

Raspberry Pi 5 üzerinde **Raspberry Pi OS (Bookworm - Debian 12)** çalışmaktadır. Bookworm sürümünde Python paketleri sanal ortam (venv) ile kurulmalıdır.

Terminali açın ve şu adımları izleyin:

### Adım 1: Sistem Güncellemesi ve Temel Paketler
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip python3-venv python3-opencv libcamera-tools
```

### Adım 2: Proje Klasörüne Gitme ve Sanal Ortam Oluşturma
```bash
cd ~/Desktop/KameraKure

# Sanal ortam oluşturma (--system-site-packages sistem kütüphanelerine erişim sağlar)
python3 -m venv --system-site-packages env

# Sanal ortamı aktif etme
source env/bin/activate
```

### Adım 3: Gerekli Python Kütüphanelerini Yükleme
```bash
pip install opencv-python numpy gpiozero lgpio
```

*(Not: Raspberry Pi 5 yeni RP1 çipini kullandığı için eski `RPi.GPIO` kütüphanesi yerine `gpiozero` ve `lgpio` kullanılır. Kodumuz buna tam uyumludur).*

### Adım 4: Programı Çalıştırma
```bash
python v1.py
```

Programdan çıkmak için kamera penceresi seçiliyken **`q`** tuşuna basabilir veya terminalde **`Ctrl + C`** yapabilirsiniz.

---

## 3. Kod Üzerindeki Hassasiyet ve Ayarlar (`v1.py`)

Yeni sürüm, DC motorların hedefe tam oturması ve aşırı savrulmayı (overshoot) önlemek için **3 Aşamalı Hassas Oransal Kontrol (P-Controller)** kullanır:

- **`DEADZONE_X` ve `DEADZONE_Y` (Varsayılan: `±20 px`)**:
  - Hedefin tam merkezde kilitleneceği **Bullseye** alanıdır. Nesne bu kutunun içine girdiğinde motorlar derhal frenlenir ve kilitlenir.
- **`FINE_ZONE_X` ve `FINE_ZONE_Y` (Varsayılan: `65 px`)**:
  - Hassas yanaşma bölgesidir. Hedefe yaklaşıldığında motor hızı otomatik olarak `MIN_SPEED` seviyesine çekilir, böylece hedefin üzerinden hızla geçip gitmesi (aşırı salınım) engellenir.
- **`MIN_SPEED` ve `MAX_SPEED` (Varsayılan: `0.35` - `0.85`)**:
  - DC motorun sürtünmeyi yenebildiği minimum hız (`0.35`) ve hedef uzaktayken hızlanacağı tavan hızdır.
- **`EMA_ALPHA` (Varsayılan: `0.70`)**:
  - Kamera gürültüsü kaynaklı piksel titremelerini filtreleyen yumuşatma katsayısıdır. Motorların titremesini önler.
- **`CONTROL_MODE = "PAN_TILT"`**:
  - `"PAN_TILT"`: Motor 1 yatay ekseni (sağ/sol), Motor 2 dikey ekseni (yukarı/aşağı) kontrol eder. Kamera küresi / gimbal mekanizmaları için idealdir.
  - `"DIFFERENTIAL"`: 2 tekerlekli araba / robot sürüşü içindir.

---

## 4. Bilgisayarda (Windows / Mac) Simülasyon Testi

Kodu Raspberry Pi'ye atmadan önce bilgisayarınızın web kamerasıyla test edebilirsiniz. Bilgisayarda GPIO donanımı olmadığı için program otomatik olarak **Simülasyon Modu**na geçer ve ekranda motorların hangi yöne gideceğini yazarak simüle eder.

---

## 5. SG90 Servo Motor Konsol Testi (`test.py`)

Konsol üzerinden doğrudan SG90 mikro servo motoru test etmek için `test.py` dosyasını kullanabilirsiniz.

### Bağlantı Şeması:
| SG90 Kablo Rengi | Raspberry Pi Pini | Açıklama |
| :--- | :--- | :--- |
| **Turuncu / Sarı** | **GPIO 18** (Fiziksel Pin 12) | PWM Sinyal Pini |
| **Kırmızı** | **5V** (Fiziksel Pin 2 veya 4) | Güç / VCC |
| **Kahverengi / Siyah**| **GND** (Fiziksel Pin 6, 9 veya 14)| Toprak Hattı |

> 💡 **İpucu**: Motorun açıda sabit dururken titremesini ve ısınmasını önlemek için sinyal otomatik kesilir (`detach`).

### Çalıştırma Seçenekleri:
```bash
# 1. Etkileşimli Konsol Menüsü (Doğrudan açı girme, süpürme, +/- adım ayarı):
python test.py

# 2. Hızlı Açı Komutu (Motoru 90 dereceye götürür):
python test.py 90

# 3. Otomatik Süpürme Testi (0° -> 180° -> 90°):
python test.py --sweep

# 4. Farklı bir GPIO pini kullanmak için (Örn: GPIO 12):
python test.py --pin 12
```

---

## 6. Iriun Webcam ile Çalıştırma (`v2.py`)

Telefonunuzu kablosuz (Wi-Fi) veya USB ile web kamerası olarak kullanmak için **Iriun Webcam** desteği `v2.py` içine entegre edilmiştir.

### Kurulum:
1. Telefonunuza **Iriun Webcam** uygulamasını kurun ve açın.
2. Bilgisayarınıza / Pi'ye **Iriun Webcam** masaüstü uygulamasını kurun.
3. Telefon ile bilgisayarın **aynı Wi-Fi ağına** bağlı olduğundan (veya USB ile bağlı) emin olun.

### Çalıştırma Komutları:
```bash
# Otomatik bağlanma (Genellikle İndeks 1 veya 0'ı dener):
python v2.py

# Belirli bir kamera indeksi belirtmek için (Örn: Kamera 1):
python v2.py --camera 1

# Sistemdeki çalışan tüm kameraları listelemek için:
python v2.py --list-cameras

# Ön/Selfie kamerası ters yönlendiriyorsa aynalama açık başlatma:
python v2.py --flip
```

> 💡 **Kamera Ekranı Kısayolları:**
> - **`f` Tuşu**: Aynalama (Flip) modunu canlı olarak açıp kapatır (Özellikle ön kamera yönü ters geliyorsa anında düzeltir).
> - **`q` Tuşu**: Programı ve motorları güvenle kapatır.

