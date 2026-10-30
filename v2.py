"""
=============================================================================
Raspberry Pi 5 / PC - Hassas Mavi Renk Takip ve Oransal DC Motor Kontrolü
Proje: KameraKure (Hassas Nesne Merkezleme Sistemi v2 - Iriun Webcam Desteği)
=============================================================================
Yenilikler ve Iriun Webcam Entegrasyonu:
1. IRIUN WEBCAM DESTEĞİ:
   - Telefonu kablosuz (Wi-Fi) veya USB ile web kamerası olarak kullanabilme.
   - Doğrudan kamera indeksi seçimi (Genellikle İndeks 1, tek kamera varsa 0).
   - Windows DirectShow (CAP_DSHOW) optimizasyonu: Iriun gecikmesini ve donmasını önler.
   - Yüksek çözünürlük normalizasyonu: Telefon 1080p veya 720p yayınlasa bile
     işlem gücünü ve takip hassasiyetini korumak için 640x480'e akıcı ölçekler.
   - Otomatik Kamera Tarama ve Kurtarma: Seçilen kamera açılmazsa sistemdeki
     diğer kameraları otomatik tespit eder.
   - Anlık Ayna (Flip) Modu: Ön/Selfie kamerası ters yönlendiriyorsa 'f' tuşuna
     basarak anında görüntüyü yatay çevirebilirsiniz.

2. ORANSAL HIZ KONTROLÜ (P-Controller): Hedefe olan uzaklığa göre motor hızı
   dinamik olarak ayarlanır. Hedef uzaktayken hızlı, yaklaştıkça yavaşlayarak
   aşırı savrulmayı (overshoot / salınım) önler.

3. 3 AŞAMALI HASSAS BÖLGE MİMARİSİ:
   - TAM MERKEZ (Bullseye): ±0 piksel olana kadar motorlar hedefe sürülür.
   - HASSAS YANAŞMA (Fine Inching): 0-65 piksel arasında minimum tork hızında
     hassas mikro yanaşma yapar.
   - HIZLI YAKLAŞMA: 65 piksel üstünde oransal hızlanma ile hedefe yaklaşır.

4. EMA TİTREŞİM FİLTRESİ (Exponential Moving Average): Kamera sensörünün 
   gürültüsünden kaynaklanan anlık piksel sıçramalarını yumuşatarak motorların
   titremesini engeller.
=============================================================================
"""

import os
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

import cv2
try:
    cv2.setLogLevel(cv2.LOG_LEVEL_ERROR)
except Exception:
    pass

import numpy as np
import time
import platform
import argparse
import sys

# =============================================================================
# 1. HASSASİYET VE SİSTEM AYARLARI
# =============================================================================

# Motor Kontrol Modu:
# 'PAN_TILT'     : Motor 1 Yatay (Pan / Sağ-Sol), Motor 2 Dikey (Tilt / Yukarı-Aşağı)
# 'DIFFERENTIAL' : 2 tekerlekli robot sürüşü (Sol Motor ve Sağ Motor)
CONTROL_MODE = "PAN_TILT"

# --- HASSAS MERKEZLEME BÖLGE AYARLARI (Piksel Cinsinden) ---
DEADZONE_X = 0  # Sapma X tam 0 olana kadar motor 1 hareket eder
DEADZONE_Y = 0  # Sapma Y tam 0 olana kadar motor 2 hareket eder

# Hassas Yanaşma Bölgesi: Bu sınırın altında motorlar minimum hızla hedefe ince ayar yapar
FINE_ZONE_X = 65
FINE_ZONE_Y = 65

# --- MOTOR HIZ AYARLARI (0.0 ile 1.0 arası) ---
MIN_SPEED = 0.35  # DC motorun sürtünmeyi yenip dönebildiği en düşük hız eşiği
MAX_SPEED = 0.85  # Hedef uzaktayken motorun çıkacağı tavan hız

# Motor Yönlerini Ters Çevirme (Kabloları değiştirmeden yön düzeltmek için)
INVERT_M1 = False
INVERT_M2 = False

# --- TİTREŞİM FİLTRESİ (EMA Smoothing) ---
# 0.0 (en yavaş tepki, sıfır titreme) ile 1.0 (anlık ham veri, filtrelenmemiş) arası
EMA_ALPHA = 0.70

# --- KAMERA VE GÖRÜNTÜ AYARLARI ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Varsayılan Kamera Seçimi:
# 1 : Laptop/PC'de dahili kamera varsa Iriun Webcam genellikle İndeks 1'dir.
# 0 : Dahili kamera yoksa veya tek kamera varsa Iriun Webcam İndeks 0'dır.
DEFAULT_CAMERA_SOURCE = 1
DEFAULT_FLIP_HORIZONTAL = False  # Başlangıçta aynalama kapalı (f tuşu ile değiştirilebilir)

# Mavi Renk HSV Aralıkları (Ortam ışığına göre optimize edilmiş geniş aralık)
LOWER_BLUE = np.array([95, 110, 50])
UPPER_BLUE = np.array([135, 255, 255])

# Algılanacak minimum piksel alanı (küçük arka plan gürültülerini filtreler)
MIN_CONTOUR_AREA = 700

# --- RASPBERRY PI 5 GPIO PİNLERİ (BCM Numaralandırma) ---
# Motor 1 (Pan / Sol Motor)
PIN_M1_IN1 = 17
PIN_M1_IN2 = 27
PIN_M1_ENA = 22  # PWM Hız pini

# Motor 2 (Tilt / Sağ Motor)
PIN_M2_IN1 = 23
PIN_M2_IN2 = 24
PIN_M2_ENB = 25  # PWM Hız pini

# =============================================================================
# 2. MOTOR SÜRÜCÜSÜ (Raspberry Pi 5 & Simülasyon Desteği)
# =============================================================================

USE_GPIO = False
motor1 = None
motor2 = None

try:
    from gpiozero import Motor
    # Raspberry Pi 5 üzerinde donanımsal/yazılımsal PWM ile motorları başlat
    motor1 = Motor(forward=PIN_M1_IN1, backward=PIN_M1_IN2, enable=PIN_M1_ENA, pwm=True)
    motor2 = Motor(forward=PIN_M2_IN1, backward=PIN_M2_IN2, enable=PIN_M2_ENB, pwm=True)
    USE_GPIO = True
    print("[BILGI] Raspberry Pi 5 GPIO ve PWM Motor Sürücüleri Başlatıldı.")
except Exception as e:
    print(f"[BILGI] GPIO kütüphanesi aktif değil ({e}).")
    print("[BILGI] SİSTEM SİMÜLASYON MODUNDA ÇALIŞIYOR (Yalnızca kamera izleme aktif).")
    USE_GPIO = False


class SimulatedMotor:
    """Bilgisayarda simülasyon testi için sanal motor sınıfı"""
    def __init__(self, name):
        self.name = name
        self.direction = "DURUYOR"
        self.speed = 0.0

    def forward(self, speed=1.0):
        self.direction = "ILERI"
        self.speed = speed

    def backward(self, speed=1.0):
        self.direction = "GERI"
        self.speed = speed

    def stop(self):
        self.direction = "DURUYOR"
        self.speed = 0.0


if not USE_GPIO:
    motor1 = SimulatedMotor("Motor 1 (Pan/Sol)")
    motor2 = SimulatedMotor("Motor 2 (Tilt/Sağ)")


def set_motor(motor, direction, speed=0.0, invert=False):
    """
    Motorun yönünü ve hassas PWM hızını ayarlar.
    direction: 1 (Artı yön), -1 (Eksi yön), 0 (Fren / Dur)
    speed: 0.0 - 1.0 arası hız
    """
    if invert:
        direction = -direction

    speed = max(0.0, min(1.0, float(speed)))

    if direction > 0 and speed > 0.01:
        try:
            motor.forward(speed)
        except TypeError:
            motor.forward()
    elif direction < 0 and speed > 0.01:
        try:
            motor.backward(speed)
        except TypeError:
            motor.backward()
    else:
        motor.stop()


def stop_all_motors():
    """Tüm motorları frenleyip durdurur."""
    motor1.stop()
    motor2.stop()


# =============================================================================
# 3. KAMERA YÖNETİMİ VE IRIUN WEBCAM BAĞLANTISI
# =============================================================================

def scan_available_cameras(max_tested=5):
    """
    Sistemdeki çalışan tüm kamera indekslerini tespit eder.
    """
    available_indices = []
    is_windows = (platform.system() == "Windows")

    for idx in range(max_tested):
        backend = cv2.CAP_DSHOW if is_windows else cv2.CAP_ANY
        cap_test = cv2.VideoCapture(idx, backend)
        if cap_test.isOpened():
            # Test karesi oku
            ret, _ = cap_test.read()
            if ret:
                available_indices.append(idx)
            cap_test.release()
    return available_indices


def open_camera_stream(requested_source):
    """
    İstenen kamera kaynağını (indeks veya URL) başlatır.
    Iriun Webcam için Windows üzerinde DirectShow (CAP_DSHOW) kullanır.
    Başarısız olursa diğer indeksleri otomatik dener.
    """
    is_windows = (platform.system() == "Windows")
    cap = None
    actual_source = requested_source

    # Sayısal indeks mi yoksa IP/URL mi?
    is_numeric = False
    try:
        source_idx = int(requested_source)
        is_numeric = True
    except ValueError:
        source_idx = requested_source

    print(f"\n[KAMERA] Kamera kaynağı başlatılıyor: {requested_source}...")

    if is_numeric:
        backend = cv2.CAP_DSHOW if is_windows else cv2.CAP_ANY
        cap = cv2.VideoCapture(source_idx, backend)
        
        # Eğer DirectShow ile açılmadıysa varsayılan backend ile tekrar dene
        if not cap.isOpened() and is_windows:
            cap = cv2.VideoCapture(source_idx)
    else:
        # IP / RTSP / HTTP Stream URL
        cap = cv2.VideoCapture(source_idx)

    # Çözünürlük ayarlarını uygula
    if cap.isOpened():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        # Hızlı ilk kare testi
        ret, _ = cap.read()
        if ret:
            print(f"[BAŞARILI] Kamera {actual_source} başarıyla bağlandı ve görüntü veriyor.")
            return cap, actual_source

    # Belirtilen kaynak açılmadıysa kurtarma (fallback) mekanizması
    print(f"[UYARI] Kamera {requested_source} açılamadı veya görüntü vermiyor.")
    print("[BİLGİ] Sistemdeki diğer kameralar taranıyor, lütfen bekleyin...")
    
    found_cams = scan_available_cameras(max_tested=5)
    if not found_cams:
        print("[HATA] Sistemde hiçbir aktif kamera bulunamadı!")
        print("  - Telefonunuzdaki Iriun Webcam uygulamasının açık olduğunu kontrol edin.")
        print("  - Bilgisayar ile telefonun aynı Wi-Fi ağına bağlı olduğundan emin olun.")
        print("  - Alternatif olarak telefonu USB kablosu ile bağlamayı deneyin.")
        return None, None

    print(f"[BİLGİ] Bulunan aktif kameralar: {found_cams}")
    
    # Kullanıcının istediği hariç bulunan ilk çalışan kamerayı dene
    for alt_idx in found_cams:
        if alt_idx != source_idx:
            print(f"[DURUM] Alternatif Kamera {alt_idx} deneniyor (Iriun Webcam olabilir)...")
            backend = cv2.CAP_DSHOW if is_windows else cv2.CAP_ANY
            cap = cv2.VideoCapture(alt_idx, backend)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
                    print(f"[BAŞARILI] Alternatif Kamera {alt_idx} bağlandı ve devreye alındı.")
                    return cap, alt_idx
            cap.release()

    return None, None


# =============================================================================
# 4. HASSAS ORANSAL KONTROL MANTIĞI
# =============================================================================

def calculate_axis_action(error, fine_zone, max_error):
    """
    Tek bir eksen için (X veya Y) TAM 0 hedefine göre yön ve oransal hız hesaplar.
    """
    abs_err = abs(error)

    # 1. Aşama: Tam 0 Sapma Kontrolü (Hata == 0 ise KİLİTLENİR)
    if abs_err == 0:
        return 0, 0.0, "TAM 0 (KILIT)"

    # Yön tayini: Hata pozitifse hedef sağda/aşağıda (1), negatifse solda/yukarıda (-1)
    direction = 1 if error > 0 else -1

    # 2. Aşama: Hassas Yanaşma Bölgesi (1 <= abs_err <= fine_zone)
    if abs_err <= fine_zone:
        fine_factor = abs_err / max(1.0, float(fine_zone))
        speed = MIN_SPEED + (0.50 - MIN_SPEED) * fine_factor
        zone_name = f"HASSAS YANASMA ({abs_err}px)"
        return direction, speed, zone_name

    # 3. Aşama: Oransal Hızlı Takip Bölgesi (> fine_zone)
    progress = (abs_err - fine_zone) / max(1.0, float(max_error - fine_zone))
    progress = max(0.0, min(1.0, progress))
    speed = 0.50 + (MAX_SPEED - 0.50) * progress
    zone_name = "HIZLI TAKIP"

    return direction, speed, zone_name


def update_precision_tracking(cx, cy, frame_w, frame_h):
    """
    Filtrelenmiş hedef koordinatlarına göre her iki motoru sürer.
    """
    center_x = frame_w // 2
    center_y = frame_h // 2

    error_x = int(round(cx - center_x))  # > 0 ise hedef sağda, < 0 ise solda
    error_y = int(round(cy - center_y))  # > 0 ise hedef aşağıda, < 0 ise yukarıda

    max_error_x = frame_w // 2
    max_error_y = frame_h // 2

    # Her iki eksen için bağımsız hesaplama (Tam 0 şartı)
    dir_x, speed_x, zone_x = calculate_axis_action(error_x, FINE_ZONE_X, max_error_x)
    dir_y, speed_y, zone_y = calculate_axis_action(error_y, FINE_ZONE_Y, max_error_y)

    is_centered_x = (error_x == 0)
    is_centered_y = (error_y == 0)
    is_fully_centered = is_centered_x and is_centered_y

    if CONTROL_MODE == "PAN_TILT":
        # Motor 1: Yatay Eksen (Pan)
        set_motor(motor1, dir_x, speed_x, INVERT_M1)

        # Motor 2: Dikey Eksen (Tilt)
        set_motor(motor2, -dir_y, speed_y, INVERT_M2)

        m1_text = "TAM 0 (KILIT)" if is_centered_x else f"{'SOL' if dir_x < 0 else 'SAG'} (%{int(speed_x*100)})"
        m2_text = "TAM 0 (KILIT)" if is_centered_y else f"{'YUKARI' if dir_y < 0 else 'ASAGI'} (%{int(speed_y*100)})"

    elif CONTROL_MODE == "DIFFERENTIAL":
        # İki Tekerlekli Diferansiyel Sürüş
        if not is_centered_x:
            if dir_x < 0:
                set_motor(motor1, -1, speed_x, INVERT_M1)
                set_motor(motor2, 1, speed_x, INVERT_M2)
                m1_text = f"SOLA DÖN (%{int(speed_x*100)})"
                m2_text = f"SOLA DÖN (%{int(speed_x*100)})"
            else:
                set_motor(motor1, 1, speed_x, INVERT_M1)
                set_motor(motor2, -1, speed_x, INVERT_M2)
                m1_text = f"SAGA DÖN (%{int(speed_x*100)})"
                m2_text = f"SAGA DÖN (%{int(speed_x*100)})"
        else:
            stop_all_motors()
            m1_text = "TAM 0 (KILIT)"
            m2_text = "TAM 0 (KILIT)"

    return m1_text, m2_text, error_x, error_y, is_fully_centered


# =============================================================================
# 5. GÖRSEL NİŞANGAH VE HUD ÇİZİMİ
# =============================================================================

def draw_precision_hud(frame, center_x, center_y, smooth_cx, smooth_cy,
                       error_x, error_y, m1_text, m2_text, is_centered,
                       target_found, fps, cam_source, flip_active):
    """
    Ekrana hassas nişangah, durum kartı ve Iriun kamera bilgilerini çizer.
    """
    h, w = frame.shape[:2]

    COLOR_LOCK = (0, 255, 0)       # Yeşil: Tam 0 Kilitlendi
    COLOR_MOVING = (0, 165, 255)   # Turuncu: Hareket Halinde (Sapma var)
    COLOR_SEARCH = (60, 60, 220)   # Kırmızı: Hedef Yok

    is_x_zero = (error_x == 0 and target_found)
    is_y_zero = (error_y == 0 and target_found)

    # 1. Hassas Yanaşma Alanı Kılavuz Çerçevesi (Fine Zone)
    fx1 = center_x - FINE_ZONE_X
    fx2 = center_x + FINE_ZONE_X
    fy1 = center_y - FINE_ZONE_Y
    fy2 = center_y + FINE_ZONE_Y
    cv2.rectangle(frame, (fx1, fy1), (fx2, fy2), (60, 60, 60), 1)

    # 2. Hassas Eksen Kilit Çizgileri
    color_axis_x = COLOR_LOCK if is_x_zero else ((120, 120, 120) if target_found else (70, 70, 70))
    color_axis_y = COLOR_LOCK if is_y_zero else ((120, 120, 120) if target_found else (70, 70, 70))

    cv2.line(frame, (center_x, fy1 - 10), (center_x, fy2 + 10), color_axis_x, 1)
    cv2.line(frame, (fx1 - 10, center_y), (fx2 + 10, center_y), color_axis_y, 1)

    # 3. Merkez Tam Sıfır Noktası
    center_color = COLOR_LOCK if is_centered else (COLOR_MOVING if target_found else COLOR_SEARCH)
    cv2.circle(frame, (center_x, center_y), 4, center_color, -1)
    if is_centered:
        cv2.circle(frame, (center_x, center_y), 14, COLOR_LOCK, 2)

    # 4. Hedef Bulunduysa Vektör ve Çember
    if target_found and smooth_cx is not None:
        target_pt = (int(smooth_cx), int(smooth_cy))
        cv2.line(frame, (center_x, center_y), target_pt, center_color, 2)
        cv2.circle(frame, target_pt, 5, (0, 0, 255), -1)
        cv2.circle(frame, target_pt, 16, center_color, 2)

    # 5. Üst Bilgi Kartı (HUD)
    cv2.rectangle(frame, (10, 10), (380, 125), (20, 20, 20), -1)
    cv2.rectangle(frame, (10, 10), (380, 125), center_color, 1)

    if is_centered:
        status_str = "HEDEF KILITLENDI (TAM 0 SAPMA)"
        status_color = COLOR_LOCK
    elif target_found:
        status_str = f"MERKEZE SURULUYOR (X:{error_x:+d}, Y:{error_y:+d})"
        status_color = COLOR_MOVING
    else:
        status_str = "MAVI HEDEF ARANIYOR"
        status_color = COLOR_SEARCH

    cv2.putText(frame, f"DURUM: {status_str}", (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2)
    cv2.putText(frame, f"Motor 1 (Pan) : {m1_text}", (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1)
    cv2.putText(frame, f"Motor 2 (Tilt): {m2_text}", (20, 82),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (240, 240, 240), 1)

    if target_found:
        badge_x = "[0 KILIT]" if is_x_zero else "[HAREKETTE]"
        badge_y = "[0 KILIT]" if is_y_zero else "[HAREKETTE]"
        cv2.putText(frame, f"dX:{error_x:+d}px {badge_x} | dY:{error_y:+d}px {badge_y}", (20, 106),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
    else:
        cv2.putText(frame, "Sapma -> Mavi Hedef Görünmüyor", (20, 106),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (150, 150, 150), 1)

    # Sağ Üst Göstergeler (Kamera, FPS ve Kontroller)
    cv2.putText(frame, f"FPS: {fps:.1f}", (w - 110, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
    cv2.putText(frame, f"Kamera: {cam_source}", (w - 150, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
    
    flip_text = "AYNA: ACIK" if flip_active else "AYNA: KAPALI"
    flip_color = (0, 255, 255) if flip_active else (160, 160, 160)
    cv2.putText(frame, f"{flip_text} [F]", (w - 150, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, flip_color, 1)

    # Alt Bilgi Şeridi
    cv2.putText(frame, "Klavye: [Q] Cikis | [F] Aynala / Cevir", (15, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)


# =============================================================================
# 6. ANA ÇALIŞMA DÖNGÜSÜ
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi 5 / PC - Iriun Webcam & Hassas Motor Takip v2"
    )
    parser.add_argument(
        "-c", "--camera",
        default=str(DEFAULT_CAMERA_SOURCE),
        help=f"Kamera indeksi veya URL (Varsayılan: {DEFAULT_CAMERA_SOURCE}). Iriun genelde 1 veya 0'dır."
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Sistemdeki tüm aktif kameraları listeler ve çıkar."
    )
    parser.add_argument(
        "--flip",
        action="store_true",
        help="Görüntüyü başlangıçta yatay olarak aynalar (Selfie kamerası için)."
    )

    args = parser.parse_args()

    # Eğer sadece kamera listeleme istendiyse
    if args.list_cameras:
        print("\nSistemdeki kameralar taranıyor...")
        cams = scan_available_cameras(max_tested=8)
        if cams:
            print(f"[SONUÇ] Aktif kamera indeksleri: {cams}")
            print("Öneri: Iriun Webcam çoğunlukla bu indekslerden biridir (örneğin 1 veya 0).")
        else:
            print("[SONUÇ] Aktif kamera bulunamadı. Telefondaki Iriun uygulamasını kontrol edin.")
        return

    # Kamerayı Aç
    cap, active_cam_source = open_camera_stream(args.camera)
    if cap is None:
        print("\n[HATA] Kamera bağlantısı sağlanamadı. Program sonlandırılıyor.")
        return

    flip_active = args.flip or DEFAULT_FLIP_HORIZONTAL

    print("=================================================================")
    print("  KameraKüre v2 - Hassas Mavi Takip & Iriun Webcam Devrede")
    print(f"  Aktif Kamera  : {active_cam_source}")
    print(f"  Ayna (Flip)   : {'Açık' if flip_active else 'Kapalı'} ('f' tuşu ile değiştirebilirsiniz)")
    print(f"  Motor Modu    : {CONTROL_MODE} | Tolerans: Tam 0 px")
    print("  Çıkmak için kamera penceresindeyken 'q' tuşuna basın.")
    print("=================================================================")

    smooth_cx = None
    smooth_cy = None
    prev_time = time.time()
    kernel = np.ones((5, 5), np.uint8)

    reconnect_attempts = 0

    try:
        while True:
            ret, frame = cap.read()

            # Kare alınamadıysa (Iriun anlık bağlantı kopması koruması)
            if not ret or frame is None:
                reconnect_attempts += 1
                if reconnect_attempts % 30 == 0:
                    print("[UYARI] Iriun Webcam'den kare bekleniyor... (Uygulamanın açık olduğunu doğrulayın)")
                time.sleep(0.03)
                if reconnect_attempts > 120:  # ~4 saniye bekledikten sonra
                    print("[UYARI] Bağlantı koptu, yeniden bağlanmayı deniyor...")
                    cap.release()
                    cap, _ = open_camera_stream(active_cam_source)
                    reconnect_attempts = 0
                continue
            
            reconnect_attempts = 0

            # 1. Çözünürlük Normalizasyonu
            # Iriun telefondan 1080p veya 720p gönderse bile işlem gecikmesini
            # önlemek ve takip ölçeğini korumak için 640x480'e ölçeklenir:
            if frame.shape[1] != FRAME_WIDTH or frame.shape[0] != FRAME_HEIGHT:
                frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT), interpolation=cv2.INTER_LINEAR)

            # 2. Aynalama (Flip) Kontrolü
            if flip_active:
                frame = cv2.flip(frame, 1)

            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2

            # 3. Görüntü Yumuşatma ve Renk Dönüşümü
            blurred = cv2.GaussianBlur(frame, (9, 9), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 4. Mavi Renk Filtresi
            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            # 5. Kontur Tespiti
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            target_found = False
            raw_cx, raw_cy = None, None
            m1_text = "HEDEF YOK"
            m2_text = "HEDEF YOK"
            error_x, error_y = 0, 0
            is_centered = False

            if contours:
                largest = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest)

                if area >= MIN_CONTOUR_AREA:
                    M = cv2.moments(largest)
                    if M["m00"] > 0:
                        raw_cx = float(M["m10"] / M["m00"])
                        raw_cy = float(M["m01"] / M["m00"])

                        # EMA Titreşim Filtresi Uygula
                        if smooth_cx is None:
                            smooth_cx, smooth_cy = raw_cx, raw_cy
                        else:
                            smooth_cx = EMA_ALPHA * raw_cx + (1.0 - EMA_ALPHA) * smooth_cx
                            smooth_cy = EMA_ALPHA * raw_cy + (1.0 - EMA_ALPHA) * smooth_cy

                        target_found = True

                        # Hassas Motor Kontrolünü Çalıştır
                        m1_text, m2_text, error_x, error_y, is_centered = update_precision_tracking(
                            smooth_cx, smooth_cy, w, h
                        )

            if not target_found:
                stop_all_motors()
                smooth_cx, smooth_cy = None, None

            # FPS Hesaplama
            curr_time = time.time()
            fps = 1.0 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            # 6. Hassas Görsel Arayüz (HUD)
            draw_precision_hud(frame, center_x, center_y, smooth_cx, smooth_cy,
                               error_x, error_y, m1_text, m2_text, is_centered,
                               target_found, fps, active_cam_source, flip_active)

            # Pencereleri göster
            cv2.imshow("Hassas Kamera Takip (KameraKure - Iriun Destekli)", frame)
            cv2.imshow("Mavi Maske", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('f'):
                flip_active = not flip_active
                print(f"[BILGI] Aynalama (Flip) modu: {'ACIK' if flip_active else 'KAPALI'}")

    except KeyboardInterrupt:
        print("\n[BILGI] Sistem durduruldu.")

    finally:
        print("[BILGI] Motorlar durduruluyor ve kamera kapatılıyor...")
        stop_all_motors()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print("[BILGI] Güvenli kapatma tamamlandı.")


if __name__ == "__main__":
    main()