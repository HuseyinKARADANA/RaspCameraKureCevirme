"""
=============================================================================
Raspberry Pi 5 - Hassas Mavi Renk Takip ve Oransal DC Motor Kontrolü
Proje: KameraKure (Hassas Nesne Merkezleme Sistemi v2)
=============================================================================
Yenilikler ve Hassasiyet Özellikleri:
1. ORANSAL HIZ KONTROLÜ (P-Controller): Hedefe olan uzaklığa göre motor hızı
   dinamik olarak ayarlanır. Hedef uzaktayken hızlı, yaklaştıkça yavaşlayarak
   aşırı savrulmayı (overshoot / salınım) önler.
2. 3 AŞAMALI HASSAS BÖLGE MİMARİSİ:
   - TAM MERKEZ (Bullseye): ±18 piksel içinde motorlar tamamen durur ve kilitlenir.
   - HASSAS YANAŞMA (Fine Inching): 18-60 piksel arasında minimum tork hızında
     hassas mikro yanaşma yapar.
   - HIZLI YAKLAŞMA: 60 piksel üstünde oransal hızlanma ile hedefe yaklaşır.
3. EMA TİTREŞİM FİLTRESİ (Exponential Moving Average): Kamera sensörünün 
   gürültüsünden kaynaklanan anlık piksel sıçramalarını yumuşatarak motorların
   titremesini engeller.
4. RASPBERRY PI 5 TAM PWM DESTEĞİ: gpiozero kütüphanesi üzerinden PWM hız kontrolü.
   Windows/Mac üzerinde çalışırken otomatik 'Simülasyon Modu' devreye girer.
=============================================================================
"""

import cv2
import numpy as np
import time
import platform

# =============================================================================
# 1. HASSASİYET VE SİSTEM AYARLARI
# =============================================================================

# Motor Kontrol Modu:
# 'PAN_TILT'     : Motor 1 Yatay (Pan / Sağ-Sol), Motor 2 Dikey (Tilt / Yukarı-Aşağı)
# 'DIFFERENTIAL' : 2 tekerlekli robot sürüşü (Sol Motor ve Sağ Motor)
CONTROL_MODE = "PAN_TILT"

# --- HASSAS MERKEZLEME BÖLGE AYARLARI (Piksel Cinsinden) ---
# KURAL: Motorlar sapma X ve Y eksenleri TAM 0 OLMADAN kilitlenmez!
DEADZONE_X = 0  # Sapma X tam 0 olana kadar motor 1 hareket eder
DEADZONE_Y = 0  # Sapma Y tam 0 olana kadar motor 2 hareket eder

# Hassas Yanaşma Bölgesi: Bu sınırın altında motorlar minimum hızla hedefe ince ayar yapar
FINE_ZONE_X = 65
FINE_ZONE_Y = 65

# --- MOTOR HIZ AYARLARI (0.0 ile 1.0 arası) ---
# MIN_SPEED: DC motorun sürtünmeyi yenip dönebildiği en düşük hız eşiği (Stall koruması)
MIN_SPEED = 0.35
# MAX_SPEED: Hedef uzaktayken motorun çıkacağı tavan hız
MAX_SPEED = 0.85

# Motor Yönlerini Ters Çevirme (Kabloları değiştirmeden yön düzeltmek için)
INVERT_M1 = False
INVERT_M2 = False

# --- TİTREŞİM FİLTRESİ (EMA Smoothing) ---
# 0.0 (en yavaş tepki, sıfır titreme) ile 1.0 (anlık ham veri, filtrelenmemiş) arası
EMA_ALPHA = 0.70

# --- KAMERA VE RENK AYARLARI ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Mavi Renk HSV Aralıkları (Ortam ışığına göre optimize edilmiş geniş aralık)
LOWER_BLUE = np.array([95, 110, 50])
UPPER_BLUE = np.array([135, 255, 255])

# Algılanacak minimum piksel alanı (küçük arka plan gürültülerini filtreler)
MIN_CONTOUR_AREA = 700

# --- RASPBERRY PI 5 GPIO PİNLERİ (BCM Numaralandırma) ---
# Motor 1 (Pan / Sol Motor)
PIN_M1_IN1 = 17
PIN_M1_IN2 = 27
PIN_M1_ENA = 22  # PWM Hız pini (L298N üzerinde jumper sökülüp buraya bağlanabilir)

# Motor 2 (Tilt / Sağ Motor)
PIN_M2_IN1 = 23
PIN_M2_IN2 = 24
PIN_M2_ENB = 25  # PWM Hız pini (L298N üzerinde jumper sökülüp buraya bağlanabilir)

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
    print("[BILGI] SİSTEM SİMÜLASYON MODUNDA ÇALIŞIYOR.")
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
# 3. HASSAS ORANSAL KONTROL MANTIĞI
# =============================================================================

def calculate_axis_action(error, fine_zone, max_error):
    """
    Tek bir eksen için (X veya Y) TAM 0 hedefine göre yön ve oransal hız hesaplar.
    KURAL: Hata tam 0 (sıfır) olmadığı sürece motor kesinlikle kilitlenmez,
    hedefi 0'a getirene kadar dönmeye devam eder.
    """
    abs_err = abs(error)

    # 1. Aşama: Tam 0 Sapma Kontrolü (Sadece ve sadece hata == 0 ise KİLİTLENİR)
    if abs_err == 0:
        return 0, 0.0, "TAM 0 (KILIT)"

    # Yön tayini: Hata pozitifse hedef sağda/aşağıda (1), negatifse solda/yukarıda (-1)
    direction = 1 if error > 0 else -1

    # 2. Aşama: Hassas Yanaşma Bölgesi (1 <= abs_err <= fine_zone)
    # Hedefe yaklaştıkça aşırı savrulmayı önleyen kademeli minimum hız
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
    X veya Y ekseninde sapma 0 olmadığı sürece motor durmaz.
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

    # Eksenler SADECE hata 0 ise kilitlenir
    is_centered_x = (error_x == 0)
    is_centered_y = (error_y == 0)
    is_fully_centered = is_centered_x and is_centered_y

    if CONTROL_MODE == "PAN_TILT":
        # Motor 1: Yatay Eksen (Pan)
        # error_x == 0 ise durur, değilse sağa/sola sürer
        set_motor(motor1, dir_x, speed_x, INVERT_M1)

        # Motor 2: Dikey Eksen (Tilt)
        # error_y == 0 ise durur, değilse yukarı/aşağı sürer
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
# 4. GÖRSEL NİŞANGAH VE HUD ÇİZİMİ
# =============================================================================

def draw_precision_hud(frame, center_x, center_y, smooth_cx, smooth_cy,
                       error_x, error_y, m1_text, m2_text, is_centered,
                       target_found, fps):
    """
    Ekrana hassas nişangah, eksen kilitlenme çizgileri ve durum bilgilerini çizer.
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

    # 2. Hassas Eksen Kilit Çizgileri (Hangi eksen 0 ise o çizgi YEŞİL yanar)
    color_axis_x = COLOR_LOCK if is_x_zero else ((120, 120, 120) if target_found else (70, 70, 70))
    color_axis_y = COLOR_LOCK if is_y_zero else ((120, 120, 120) if target_found else (70, 70, 70))

    # Dikey Eksen Çizgisi (X = 0 hizası)
    cv2.line(frame, (center_x, fy1 - 10), (center_x, fy2 + 10), color_axis_x, 1)
    # Yatay Eksen Çizgisi (Y = 0 hizası)
    cv2.line(frame, (fx1 - 10, center_y), (fx2 + 10, center_y), color_axis_y, 1)

    # 3. Merkez Tam Sıfır Noktası
    center_color = COLOR_LOCK if is_centered else (COLOR_MOVING if target_found else COLOR_SEARCH)
    cv2.circle(frame, (center_x, center_y), 4, center_color, -1)
    if is_centered:
        # Tam 0'a ulaşıldığında yeşil hedef kilidi halkası
        cv2.circle(frame, (center_x, center_y), 14, COLOR_LOCK, 2)

    # 4. Hedef Bulunduysa Vektör ve Çember
    if target_found and smooth_cx is not None:
        target_pt = (int(smooth_cx), int(smooth_cy))
        cv2.line(frame, (center_x, center_y), target_pt, center_color, 2)
        cv2.circle(frame, target_pt, 5, (0, 0, 255), -1)
        cv2.circle(frame, target_pt, 16, center_color, 2)

    # 5. Üst Bilgi Kartı (HUD)
    cv2.rectangle(frame, (10, 10), (370, 120), (20, 20, 20), -1)
    cv2.rectangle(frame, (10, 10), (370, 120), center_color, 1)

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

    # Sağ Üst Göstergeler
    cv2.putText(frame, f"FPS: {fps:.1f}", (w - 110, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 255, 0), 2)
    cv2.putText(frame, "Hedef Kriteri: Tam 0 px", (w - 180, 55),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)


# =============================================================================
# 5. ANA ÇALIŞMA DÖNGÜSÜ
# =============================================================================

def main():
    # Kamerayı başlat (Windows'ta DirectShow ile anında açılır, Pi'de standart v4l2)
    if platform.system() == "Windows":
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(0)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    if not cap.isOpened():
        print("[HATA] Kamera açılamadı! Lütfen bağlantıyı kontrol edin.")
        return

    print("=========================================================")
    print("Hassas Mavi Renk Takip ve Merkezleme Sistemi Devrede.")
    print(f"Mod: {CONTROL_MODE} | Hassasiyet: +/-{DEADZONE_X}px")
    print("Çıkmak için kamera penceresinde 'q' tuşuna basın.")
    print("=========================================================")

    # Titreşim Yumuşatma (EMA) Değişkenleri
    smooth_cx = None
    smooth_cy = None

    prev_time = time.time()
    kernel = np.ones((5, 5), np.uint8)

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.01)
                continue

            h, w = frame.shape[:2]
            center_x, center_y = w // 2, h // 2

            # 1. Görüntü Yumuşatma ve Renk Dönüşümü
            blurred = cv2.GaussianBlur(frame, (9, 9), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            # 2. Mavi Renk Filtresi
            mask = cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            # 3. Kontur Tespiti
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
                # Hedef yoksa motorları hemen durdur ve yumuşatma değerlerini sıfırla
                stop_all_motors()
                smooth_cx, smooth_cy = None, None

            # FPS Hesaplama
            curr_time = time.time()
            fps = 1.0 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            # 4. Hassas Görsel Arayüz (HUD)
            draw_precision_hud(frame, center_x, center_y, smooth_cx, smooth_cy,
                               error_x, error_y, m1_text, m2_text, is_centered,
                               target_found, fps)

            # Pencereleri göster
            cv2.imshow("Hassas Kamera Takip (KameraKure)", frame)
            cv2.imshow("Mavi Maske", mask)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n[BILGI] Sistem durduruldu.")

    finally:
        print("[BILGI] Motorlar durduruluyor ve kamera kapatılıyor...")
        stop_all_motors()
        cap.release()
        cv2.destroyAllWindows()
        print("[BILGI] Güvenli kapatma tamamlandı.")


if __name__ == "__main__":
    main()