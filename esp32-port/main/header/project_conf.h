#ifndef PROJECT_CONF_H
#define PROJECT_CONF_H

#include <stdio.h>
#include <stdint.h>

// ============================================================================
// Bu dosya `dev` branch'teki blink/header/project_conf.h dosyasindan
// ESP32-C3'e uyarlanmistir.
//
// DEV'DEN BILEREK CIKARILANLAR (sessizce atlanmadi, not dusuluyor):
// - HARDWARE_VERSION: dev'de bu deger THRESHOLD_PIN/STATUS_LED_PIN'in
//   birbirini dislayip dislamayacagini kontrol ediyordu. Bizim kartta ikisi
//   de ayri, gercek pin oldugu icin bu mantik gerekmiyor - kaldirildi
//   (bkz. defines.h'deki THRESHOLD_PIN/STATUS_LED_PIN yorumu).
// - WITHOUT_BOOTLOADER: RP2040/Pico-SDK'ya ozel "bootloader'siz flash"
//   deseni, ESP32-C3'te anlami yok, kaldirildi.
// ============================================================================

// ⚠️ ACIK KARAR - PLACEHOLDER DEGER: dev'de her cihaza ozel, derleme
// zamaninda koda gomulen bir seri no vardi ("612400080"). ESP32-C3'te seri
// numarasinin gercekte NASIL verilecegi (her cihaz icin ayri mi derlenecek?
// NVS'e mi yazilacak? efuse mi kullanilacak?) henuz karara baglanmadi -
// hocaya sorulmali. Simdilik dev'deki gibi ayni yontem (derleme zamani sabit)
// korunuyor, PLACEHOLDER bir degerle.
//
// BURASI ARTIK TEK KAYNAK: seri no flash'a yazilmiyor, "serial_num"
// partition'i hic okunmuyor. Yani bu satiri degistirip firmware yuklemek
// cihazin seri numarasini degistirmeye yeter (eskiden o sektor bir kez
// yazildiktan sonra makro degisikligi ise yaramiyordu, sektoru elle silmek
// gerekiyordu). Uzunluk SERIAL_NUMBER_SIZE (9) olmali - defines.h'deki
// _Static_assert bunu derleme zamaninda kontrol ediyor.
#define DEVICE_SERIAL_NUMBER "612600001"

// Device Password (will be written to flash)
#define DEVICE_PASSWORD "12345678"
// Device software version number
// TODO: ESP32-C3 portu icin surum numarasi netlesince guncellenecek
// NOT: dev'deki son commit ile V1.4.0 -> V1.5.0 oldu, port da ayni degere
// cekildi. Onceki "-OTATEST" eki OTA testinde iki surumu ayirt etmek icin
// gecici olarak konmustu, dev ile hizalanirken kaldirildi.
#define SOFTWARE_VERSION "V1.5.0"
// production date of device (yy-mm-dd)
#define PRODUCTION_DATE "26-08-17"
// Debugs
// ⚠️ Kullanicinin istegiyle 0 yapildi - monitorde PRINTF ciktisi istenmiyor.
#define DEBUG 1

// vrms multiplier value
// KARAR: 150 -> 148.8 olarak guncellendi. Eski deger hocanin "ayni kalsin,
// olmazsa degistiririz" dedigi 150'ydi; ekip arkadasi gercek direnc
// degerlerinden (10Mohm/68Kohm) 148.8 hesapladi ve adc_vrms_test'teki
// dogrulanmis/son surum kodda bu degeri kullandi - "buna gore yaz" talimatiyla
// birlikte port kodunda da benimsendi (bkz. CLAUDE.md Asama 2 Madde 5-6).
// Rapor icin: hocaya resmi onay/rapor notu olarak iletilmesi hala faydali,
// ama port kodu acisindan artik bloklayici degil.
#define VRMS_MULTIPLICATION_VALUE 148.8f

// watchdog timeout ms to reset device
// dev branch'te bu deger RP2040'in donanim watchdog'unun 24-bit sayac limitine
// (~8388ms) gore secilmisti. ESP32-C3'un Task Watchdog Timer'i (esp_task_wdt)
// bu donanimsal kisiti tasimiyor, ama davranissal tutarlilik icin ayni
// degerlerle basliyoruz, gerekirse ayarlariz.
#define WATCHDOG_TIMEOUT_MS 8000
#define WATCHDOG_CHECK_PERIOD_MS 5000

// RX Buffer Size
#define RX_BUFFER_SIZE 256
// identification response buffer size
#define IDENTIFICATION_RESPONSE_BUFFER_SIZE 64
// meter identify parameters
#define METER_VERSION 2
#define METER_MAX_SUPPORTED_BAUDRATE 6
#define METER_FLAG_CODE "ALP"
// max message retry count
#define MAX_MESSAGE_RETRY_COUNT 3
// request modes
#define REQUEST_MODE_SHORT_READ 0x36
#define REQUEST_MODE_LONG_READ 0x30
#define REQUEST_MODE_PROGRAMMING 0x31

// Ozellik anahtarlari (feature flags) - dev branch'teki gibi, derleme
// zamaninda her ozellik ayri ayri acilip kapanabiliyor
#define CONF_LOAD_PROFILE_ENABLED 1
#define CONF_TIME_SET_ENABLED 1
#define CONF_DATE_SET_ENABLED 1
#define CONF_PRODUCTION_INFO_ENABLED 1
#define CONF_THRESHOLD_ENABLED 1
// dev branch'te bu varsayilan KAPALI'ydi (0) ve HARDWARE_VERSION'a gore
// STATUS_LED_PIN ile birbirini disliyordu. Bizim karttaki gercek Threshold
// pin (GPIO6) ayri, gercek bir donanim oldugu icin burada ACIK (1) - bkz.
// CLAUDE.md "Kesinlesmis kararlar": Threshold pin + Status LED ikisi de
// kullanilacak, ayri ayri.
#define CONF_THRESHOLD_PIN_ENABLED 1
#define CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED 0
#define CONF_TIME_READ_ENABLED 1
#define CONF_DATE_READ_ENABLED 1
#define CONF_SERIAL_NUMBER_READ_ENABLED 1
#define CONF_VRMS_MAX_READ_ENABLED 1
#define CONF_VRMS_MIN_READ_ENABLED 1
#define CONF_VRMS_MEAN_READ_ENABLED 1
#define CONF_RESET_DATES_READ_ENABLED 1
#define CONF_THRESHOLD_OBIS_ENABLED 1

// LED PIN Hata Kodlari (dev branch'ten, blink-pattern ile gosteriliyor)
#define LED_ERROR_CODE_UART_NOT_READABLE 1
#define LED_ERROR_CODE_MESSAGE_TIMEOUT 2
#define LED_ERROR_CODE_INVALID_REQUEST_MODE 3
#define LED_ERROR_CODE_INVALID_SERIAL_NUMBER 4
#define LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN 5
#define LED_ERROR_CODE_FIFO_MUTEX_NOT_TAKEN 6
#define LED_ERROR_CODE_VRMS_VALUES_MUTEX_NOT_TAKEN 7
#define LED_ERROR_CODE_VRMS_THRESHOLD_MUTEX_NOT_TAKEN 8
#define LED_ERROR_CODE_THRESHOLD_SET_MUTEX_NOT_TAKEN 9
#define LED_ERROR_CODE_RX_BUFFER_OVERFLOW_ISR 10
#define LED_ERROR_CODE_STACK_OVERFLOW 11
// Flash'taki sektor numarasi / esik degeri gecerli araligin disinda cikti ve
// guvenli degere cekildi. Kayitlarin bir kismi kaybolmus olabilir.
#define LED_ERROR_CODE_FLASH_METADATA_CORRUPT 12

// indexed obis configuration
#define THRESHOLD_RECORD_OBIS_COUNT 10
#define RESET_DATES_OBIS_COUNT 12

// Watchdog Bits - hangi task'in "hala hayattayim" bayragini tuttugu
#define WDT_FLAG_ADC_SAMPLE (1 << 0)
#define WDT_FLAG_ADC_READ (1 << 1)
#define WDT_FLAG_UART (1 << 2)

#define WDT_ALL_TASKS_OK (WDT_FLAG_ADC_SAMPLE | WDT_FLAG_ADC_READ | WDT_FLAG_UART)
extern volatile uint32_t task_health_flags;

// DEBUG MACRO
#if DEBUG
#define PRINTF(x, ...) printf(x, ##__VA_ARGS__)
#else
#define PRINTF(x, ...)
#endif

#endif
