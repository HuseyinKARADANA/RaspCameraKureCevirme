#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
Raspberry Pi (Pi 5, Pi 4, Pi 3) - SG90 Servo Motor Konsol Kontrol Yazılımı
Proje: KameraKure
=============================================================================

SG90 Mikro Servo Kablo Bağlantı Şeması:
-----------------------------------------------------------------------------
1. TURUNCU / SARI (Sinyal / PWM) : GPIO 18 (Fiziksel Pin 12)
2. KIRMIZI (VCC / +5V)           : Raspberry Pi 5V (Fiziksel Pin 2 veya 4)
3. KAHVERENGİ / SİYAH (GND)      : Raspberry Pi GND (Fiziksel Pin 6, 9 veya 14)

Önemli Güvenlik Notu:
-----------------------------------------------------------------------------
- Tek bir SG90 servo motor yüksüz test için Raspberry Pi'nin 5V pininden beslenebilir.
- Motor zorlandığında veya mekanik bir yüke bağlandığında anlık yüksek akım çeker.
  Eğer Pi yeniden başlarsa (reset atarsa), servoyu harici 5V bir güç kaynağından
  besleyin ve harici kaynağın GND kutbunu Pi'nin GND pinine bağlayın (Ortak GND).

Kullanım:
-----------------------------------------------------------------------------
1. Doğrudan Konsol Menüsü ile Çalıştırma:
   python test.py

2. Komut Satırından Hızlı Açı Verme:
   python test.py 90            (Servoyu 90 dereceye çevirir)
   python test.py --sweep       (0° -> 180° arası otomatik süpürme testi yapar)
   python test.py --pin 18      (Farklı bir GPIO pini belirtmek için)
=============================================================================
"""

import sys
import time
import argparse
import platform

# =============================================================================
# 1. SERVO VE SİSTEM AYARLARI
# =============================================================================
DEFAULT_PIN = 18             # BCM GPIO 18 (Fiziksel Pin 12)
MIN_ANGLE = 0                # Minimum açı
MAX_ANGLE = 180              # Maksimum açı
MIN_PULSE_WIDTH = 0.0005     # 0.5 ms (SG90 için 0 derece darbe genişliği)
MAX_PULSE_WIDTH = 0.0024     # 2.4 ms (SG90 için 180 derece darbe genişliği)

# =============================================================================
# 2. DONANIM VE SİMÜLASYON SÜRÜCÜSÜ
# =============================================================================
IS_SIMULATION = False
servo = None


class SimulatedServo:
    """Bilgisayarda (Windows/Mac) test edebilmek için sanal servo sınıfı"""
    def __init__(self, pin):
        self.pin = pin
        self._angle = 90
        print(f"[SİMÜLASYON] Sanal Servo GPIO {pin} üzerinde başlatıldı.")

    @property
    def angle(self):
        return self._angle

    @angle.setter
    def angle(self, val):
        if val is None:
            self._angle = None
            return
        self._angle = max(MIN_ANGLE, min(MAX_ANGLE, float(val)))

    def detach(self):
        self._angle = None

    def close(self):
        pass


def init_servo(gpio_pin):
    """
    Belirtilen GPIO pininde servoyu başlatır.
    Raspberry Pi donanımı yoksa otomatik simülasyon moduna geçer.
    """
    global IS_SIMULATION, servo
    try:
        from gpiozero import AngularServo
        servo = AngularServo(
            gpio_pin,
            min_angle=MIN_ANGLE,
            max_angle=MAX_ANGLE,
            min_pulse_width=MIN_PULSE_WIDTH,
            max_pulse_width=MAX_PULSE_WIDTH
        )
        IS_SIMULATION = False
        print(f"[BAŞARILI] Raspberry Pi GPIO {gpio_pin} pininde SG90 Servo devrede.")
    except Exception as e:
        print(f"[UYARI] Gerçek GPIO kütüphanesi başlatılamadı ({e}).")
        print("[BİLGİ] Sistem SİMÜLASYON MODUNDA çalışıyor (Gerçek motor hareket etmeyecek).")
        print("[İPUCU] Raspberry Pi 5 üzerindeyseniz: 'pip install gpiozero lgpio' kurduğunuzdan emin olun.\n")
        servo = SimulatedServo(gpio_pin)
        IS_SIMULATION = True
    return servo


# =============================================================================
# 3. YARDIMCI VE GÖRSEL FONKSİYONLAR
# =============================================================================
def draw_angle_bar(angle, length=24):
    """Konsolda açıyı görselleştiren dinamik ilerleme çubuğu üretir."""
    if angle is None:
        return "[ BOSTA / DETACHED ]"
    ratio = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE)
    pos = int(round(ratio * (length - 1)))
    bar = ["-"] * length
    bar[pos] = "#"
    return f"[{''.join(bar)}] {angle:5.1f} derece"


def set_servo_angle(target_angle, quiet=False):
    """
    Servoyu hedef açıya götürür ve konsolda durum gösterir.
    """
    target_angle = max(MIN_ANGLE, min(MAX_ANGLE, float(target_angle)))
    servo.angle = target_angle
    if not quiet:
        mode_tag = "[SİMÜLASYON]" if IS_SIMULATION else "[DONANIM]"
        print(f" {mode_tag} Motor Açı: {draw_angle_bar(target_angle)}")
    # Servonun pozisyona ulaşabilmesi için kısa bir bekleme
    time.sleep(0.35)


def detach_servo():
    """
    PWM sinyalini keser. Bu işlem motorun titremesini, ısınmasını
    ve gereksiz akım çekmesini önler.
    """
    try:
        servo.detach()
    except Exception:
        try:
            servo.angle = None
        except Exception:
            pass
    mode_tag = "[SİMÜLASYON]" if IS_SIMULATION else "[DONANIM]"
    print(f" {mode_tag} Motor sinyali kesildi (Boşta / Detached). Titreme ve ısınma durduruldu.")


# =============================================================================
# 4. TEST VE HAREKET MODLARI
# =============================================================================
def run_sweep_test(step=15, delay=0.15):
    """
    0 dereceden 180 dereceye ve ardından merkeze (90°) yumuşak geçiş testi.
    """
    print("\n>>> OTOMATİK SÜPÜRME (SWEEP) TESTİ BAŞLATILIYOR <<<")
    print("0° -> 180° -> 90° yönünde kademeli hareket yapılıyor...")
    
    # 0'dan 180'e
    for ang in range(0, 181, step):
        servo.angle = ang
        print(f"\r İleri : {draw_angle_bar(ang)}", end="", flush=True)
        time.sleep(delay)
    print()

    time.sleep(0.5)

    # 180'den 0'a
    for ang in range(180, -1, -step):
        servo.angle = ang
        print(f"\r Geri  : {draw_angle_bar(ang)}", end="", flush=True)
        time.sleep(delay)
    print()

    time.sleep(0.5)

    # Merkeze dönüş (90)
    for ang in range(0, 91, step):
        servo.angle = ang
        print(f"\r Merkez: {draw_angle_bar(ang)}", end="", flush=True)
        time.sleep(delay)
    print("\n>>> Süpürme testi tamamlandı. Motor merkezde (90°).\n")


def run_continuous_sweep():
    """
    Kullanıcı Ctrl+C yapana kadar sürekli sağa sola salınım testi.
    """
    print("\n>>> SÜREKLİ SALINIM TESTİ BAŞLADI (Durdurmak için Ctrl+C yapın) <<<")
    try:
        while True:
            # 0 -> 180
            for ang in range(0, 181, 10):
                servo.angle = ang
                print(f"\r Salınıyor [>>>]: {draw_angle_bar(ang)}", end="", flush=True)
                time.sleep(0.08)
            time.sleep(0.2)
            # 180 -> 0
            for ang in range(180, -1, -10):
                servo.angle = ang
                print(f"\r Salınıyor [<<<]: {draw_angle_bar(ang)}", end="", flush=True)
                time.sleep(0.08)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n\n[BİLGİ] Salınım testi durduruldu.")
        set_servo_angle(90)


def run_step_mode():
    """
    Klavye üzerinden '+' (sağ) ve '-' (sol) tuşlarıyla adım adım kontrol modu.
    """
    current = servo.angle if servo.angle is not None else 90.0
    print("\n>>> ADIM ADIM İNCE AYAR MODU <<<")
    print("Komutlar:")
    print("  '+' veya 'd' : Açıyı 5° artırır")
    print("  '-' veya 'a' : Açıyı 5° azaltır")
    print("  '++'         : Açıyı 15° artırır")
    print("  '--'         : Açıyı 15° azaltır")
    print("  'm'          : 90° Merkeze al")
    print("  'q'          : Bu moddan çık")
    print("---------------------------------------------")

    while True:
        prompt = input(f"Mevcut: {current:.1f}° | Komut girin (+ / - / m / q): ").strip().lower()
        if prompt in ['q', 'cikis', 'exit']:
            break
        elif prompt in ['+', 'd']:
            current = min(180.0, current + 5.0)
        elif prompt in ['-', 'a']:
            current = max(0.0, current - 5.0)
        elif prompt == '++':
            current = min(180.0, current + 15.0)
        elif prompt == '--':
            current = max(0.0, current - 15.0)
        elif prompt == 'm':
            current = 90.0
        else:
            print("Geçersiz tuş! (+, -, ++, --, m veya q kullanın)")
            continue

        set_servo_angle(current)


# =============================================================================
# 5. ETKİLEŞİMLİ KONSOL MENÜSÜ
# =============================================================================
def print_banner(pin):
    print("=" * 68)
    print("       RASPBERRY PI - SG90 SERVO MOTOR KONSOL KONTROL PANELİ       ")
    print("=" * 68)
    print(f" Aktif Pin           : GPIO {pin} (BCM) -> Raspberry Pi Pin 12")
    print(f" Açı Sınırları       : {MIN_ANGLE}° - {MAX_ANGLE}°")
    print(f" Çalışma Modu        : {'SİMÜLASYON (Donanım Yok)' if IS_SIMULATION else 'GERÇEK DONANIM (Raspberry Pi)'}")
    print("-" * 68)
    print(" SG90 Bağlantı Hatırlatması:")
    print("   [Kablo Rengi]           [Raspberry Pi Pini]")
    print("   Turuncu / Sarı (PWM) -> GPIO 18 (Fiziksel Pin 12)")
    print("   Kırmızı        (5V)  -> 5V      (Fiziksel Pin 2 veya 4)")
    print("   Kahve / Siyah  (GND) -> GND     (Fiziksel Pin 6, 9 veya 14)")
    print("=" * 68)


def interactive_menu():
    """Kullanıcının konsoldan motoru kolayca yönetmesini sağlayan ana menü."""
    # Başlangıçta güvenli 90 derece (orta konum)
    print("\nMotor başlangıç olarak merkeze (90°) konumlandırılıyor...")
    set_servo_angle(90)

    while True:
        print("\n--- İŞLEM SEÇENEKLERİ ---")
        print(" [0 - 180] : Doğrudan herhangi bir açı girin (Örn: 0, 45, 90, 135, 180)")
        print("    [1]    : Otomatik Süpürme Testi (0° -> 180° -> 90°)")
        print("    [2]    : Adım Adım İnce Ayar (+/- tuşları ile)")
        print("    [3]    : Sürekli Sağa-Sola Salınım Testi")
        print("    [4]    : Merkeze Al (90°)")
        print("    [5]    : Motoru Boşa Al / Sinyali Kes (Isınma & titremeyi durdurur)")
        print("    [q]    : Güvenli Çıkış")
        print("-" * 68)

        secim = input("Seçiminiz veya Açı Değeri: ").strip().lower()

        if secim in ['q', 'exit', 'cikis', 'quit']:
            print("\nProgram kapatılıyor...")
            break
        elif secim == '1':
            run_sweep_test()
        elif secim == '2':
            run_step_mode()
        elif secim == '3':
            run_continuous_sweep()
        elif secim == '4':
            set_servo_angle(90)
        elif secim == '5':
            detach_servo()
        else:
            # Sayısal bir açı girildiyse
            try:
                val = float(secim)
                if MIN_ANGLE <= val <= MAX_ANGLE:
                    set_servo_angle(val)
                else:
                    print(f"[HATA] Lütfen {MIN_ANGLE} ile {MAX_ANGLE} arasında bir açı değeri girin!")
            except ValueError:
                print(f"[HATA] Geçersiz giriş: '{secim}'. Bir sayı veya menü numarası girin.")


# =============================================================================
# 6. ANA GİRİŞ NOKTASI (CLI & PARSER)
# =============================================================================
def main():
    parser = argparse.ArgumentParser(
        description="Raspberry Pi SG90 Servo Motor Konsol Kontrol Programı"
    )
    parser.add_argument(
        "angle",
        nargs="?",
        type=float,
        default=None,
        help="Doğrudan gitmek istenen açı (0 - 180). Belirtilmezse etkileşimli menü açılır."
    )
    parser.add_argument(
        "--pin",
        type=int,
        default=DEFAULT_PIN,
        help=f"Servo sinyal pini (BCM numaralandırması, varsayılan: {DEFAULT_PIN})"
    )
    parser.add_argument(
        "--sweep",
        action="store_true",
        help="Otomatik 0-180 süpürme testi yapıp programdan çıkar."
    )
    parser.add_argument(
        "--continuous",
        action="store_true",
        help="Sürekli salınım modunda çalıştırır."
    )

    args = parser.parse_args()

    # Servoyu başlat
    init_servo(args.pin)

    try:
        # CLI parametre kontrolü
        if args.sweep:
            run_sweep_test()
        elif args.continuous:
            run_continuous_sweep()
        elif args.angle is not None:
            if MIN_ANGLE <= args.angle <= MAX_ANGLE:
                print(f"Motor {args.angle}° konumuna getiriliyor...")
                set_servo_angle(args.angle)
                print("Pozisyon koruma için 1 saniye bekleniyor...")
                time.sleep(1.0)
                detach_servo()
            else:
                print(f"[HATA] Açı {MIN_ANGLE} ile {MAX_ANGLE} arasında olmalıdır!")
        else:
            # Hiçbir argüman verilmediyse doğrudan konsol menüsünü aç
            print_banner(args.pin)
            interactive_menu()

    except KeyboardInterrupt:
        print("\n[BİLGİ] Kullanıcı tarafından durduruldu (Ctrl+C).")

    finally:
        print("\nMotor güvenli duruma alınıyor...")
        detach_servo()
        try:
            servo.close()
        except Exception:
            pass
        print("[TAMAMLANDI] Güvenli kapatma sağlandı. İyi çalışmalar!")


if __name__ == "__main__":
    main()
