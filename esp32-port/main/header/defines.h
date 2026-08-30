#ifndef DEFINES_H
#define DEFINES_H
#include "header/project_conf.h"

// ============================================================================
// Bu dosya `dev` branch'teki blink/header/defines.h dosyasindan ESP32-C3'e
// uyarlanmistir. RP2040 pinleri yerine hocadan teyit edilmis ESP32-C3-MINI-1
// pinleri kullanilir (bkz. CLAUDE.md "Pin haritasi" tablosu).
// ============================================================================

// FLASH BELLEK HARITASI
// ESP32-C3'te ham adres ofsetleri yerine ESP-IDF partition table + esp_partition
// API'si kullaniliyor - gercek adresler artik partitions.csv'de tanimli,
// asagidaki isimler o partition'lari bulmak icin kullaniliyor
// (esp_partition_find_first(ESP_PARTITION_TYPE_DATA, 0x40, ISIM)).
// dev branch'teki eski RP2040 haritasi ile eslesme:
//   Serial Number -> "serial_num" (4KB)
//   Load Profile "son sektor" takibi -> "lp_sector" (4KB)
//   Load Profile Records -> "load_profile" (12 sektor, 48kB)
//   Threshold Parametreleri -> "threshold_prm" (4KB)
//   Threshold Records -> "threshold_rec" (16 sektor, 64kB)
//   Reset/Acilis Kayitlari -> "reset_dates" (4KB)
//   Sudden Amplitude Change Records -> "amp_change" (100 sektor, 400kB)
#define FLASH_SECTOR_SIZE 4096
#define FLASH_RECORD_SIZE 16
// Tum ozel veri partition'larimiz icin ortak subtype (flash_test'te 0x40
// kullanmistik, ayni degeri koruyoruz - isimler zaten benzersiz oldugu icin
// sorun yok)
#define CUSTOM_PARTITION_SUBTYPE 0x40
#define FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT 12
// load_profile partition'inin tamami (12 sektor x 4096 = 49152 byte, 0xC000)
#define FLASH_LOAD_PROFILE_RECORD_AREA_SIZE (FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT * FLASH_SECTOR_SIZE)
#define FLASH_THRESHOLD_RECORDS_SECTOR_COUNT 16
#define FLASH_AMPLITUDE_RECORDS_TOTAL_SECTOR 100
// esik kayit alanindaki sektor / slot sayilari (halka tampon)
// dev'de TH_RECORD_SECTOR_COUNT, FLASH_THRESHOLD_RECORDS_SIZE / FLASH_SECTOR_SIZE
// olarak yaziliyor; ESP'de sektor sayisi zaten dogrudan tanimli.
#define TH_RECORD_SECTOR_COUNT FLASH_THRESHOLD_RECORDS_SECTOR_COUNT
#define TH_RECORDS_PER_SECTOR (FLASH_SECTOR_SIZE / FLASH_RECORD_SIZE)
#define TH_RECORD_SLOT_COUNT (TH_RECORD_SECTOR_COUNT * TH_RECORDS_PER_SECTOR)
#define SERIAL_NUMBER_SIZE 9
#define SERIAL_NUMBER_FLAG_SIZE 3
// Seri no karsilastirmasi (uart.c/control_serial_number) uzunlugu sabit
// SERIAL_NUMBER_SIZE olarak aldigi icin, makronun bu uzunlukta olmasi
// ZORUNLU. Farkli uzunlukta bir seri no yazilirsa protokolde sessizce yanlis
// eslesme olmasin diye derleme zamaninda yakalaniyor.
_Static_assert(sizeof(DEVICE_SERIAL_NUMBER) - 1 == SERIAL_NUMBER_SIZE,
               "DEVICE_SERIAL_NUMBER tam olarak SERIAL_NUMBER_SIZE (9) karakter olmali!");

// ⚠️ "serial_num" partition'i ARTIK KULLANILMIYOR: seri no flash'a yazilmiyor,
// dogrudan DEVICE_SERIAL_NUMBER makrosundan geliyor. Partition tablodan
// SILINMEDI - silinseydi arkasindaki butun partition'larin offset'leri
// kayardi ve sahadaki kartlarin kayitlari okunamaz hale gelirdi. Bu etiket
// sadece o alanin hangi partition oldugunu belgelemek icin duruyor.
#define PARTITION_LABEL_SERIAL_NUM "serial_num"
#define PARTITION_LABEL_LP_SECTOR "lp_sector"
#define PARTITION_LABEL_LOAD_PROFILE "load_profile"
#define PARTITION_LABEL_THRESHOLD_PRM "threshold_prm"
#define PARTITION_LABEL_THRESHOLD_REC "threshold_rec"
#define PARTITION_LABEL_RESET_DATES "reset_dates"
#define PARTITION_LABEL_AMP_CHANGE "amp_change"

// MESAJLAR (dev branch'ten, protokol degismiyor)
// Request Message Without Serial Number and Flag:  /?!\r\n                     -> Length: 5
// Request Message Without Serial Number:           /?ALP!\r\n                  -> Length: 8
// Request Message with Serial Number and Flag:     /?ALP612400001!\r\n         -> Length: 17
// Reboot device message                            /?RBTDVC?\r\n               -> Length: 11
// Reset To Factory Settings message                /?RSTFS?\r\n                -> Length: 10
// Acknowledgement Message:                         [ACK]0ZX\r\n                -> Length: 6
// Password:                                        [SOH]P1[STX](12345678)[ETX][BCC]
// Load Profile with Date:                          [SOH]R2[STX]P.01(24-07-13,13:00;24-07-14,14:00)[ETX][BCC]
// Load Profile without Date:                       [SOH]R2[STX]P.01(;)[ETX][BCC]
// Time Set:                                        [SOH]W2[STX]0.9.1(13:00:00)[ETX][BCC]
// Date Set:                                        [SOH]W2[STX]0.9.2(24-07-15)[ETX][BCC]
// Production Information:                          [SOH]R2[STX]96.1.3()[ETX][BCC]
// Set Threshold Value:                             [SOH]W2[STX]96.3.12(000)[ETX][BCC]
// Set Threshold PIN:                                [SOH]W2[STX]T.P.1()[ETX][BCC]
// Get Sudden Amplitude Change Records               [SOH]R2[STX]9.9.0()[ETX][BCC]
// Read Current Time                                 [SOH]R2[STX]0.9.1()[ETX][BCC]
// Read Current Date                                 [SOH]R2[STX]0.9.2()[ETX][BCC]
// Read Serial Number                                [SOH]R2[STX]0.0.0()[ETX][BCC]
// Read Last VRMS Max                                [SOH]R2[STX]32.7.0()[ETX][BCC]
// Read Last VRMS Min                                [SOH]R2[STX]52.7.0()[ETX][BCC]
// Read Last VRMS Mean                               [SOH]R2[STX]72.7.0()[ETX][BCC]
// End Connection:                                    [SOH]B0[ETX]q             -> Length: 5

// UART / RS485 TANIMLARI
// TEYITLI: rs485_test projesinde dogrulandi (bkz. CLAUDE.md Asama 2 Madde 2)
#define UART_PORT_NUM UART_NUM_1
#define UART0_TX_PIN 21
#define UART0_RX_PIN 20
// Baud Rate for UART. Protokol geregi baslangicta 300 bit/saniye.
#define BAUD_RATE 300
// Data format: 7 data bit (ASCII), 1 stop bit, cift (even) parite
#define DATA_BITS UART_DATA_7_BITS
#define STOP_BITS UART_STOP_BITS_1
#define PARITY UART_PARITY_EVEN
// Task yigin (stack) boyutlari - baslangic degerleri, ESP-IDF'te gerekirse ayarlanacak
#define ADC_READ_TASK_STACK_SIZE (4 * 1024)
// 2*1024'ten 4*1024'e cikarildi: send_reset_dates()'teki 4096 byte'lik
// dizi static yapilip stack'ten cikarildi (bkz. uart.c'deki not), ama
// UARTTask'in derin cagri zincirleri (checkListeningData -> handler ->
// sendXXX -> PRINTF) icin ek guvenlik payi olarak stack yine de buyutuldu.
#define UART_TASK_STACK_SIZE (4 * 1024)
#define WRITE_DEBUG_TASK_STACK_SIZE (2 * 1024)
#define RESET_TASK_STACK_SIZE (2 * 1024)
#define ADC_SAMPLE_TASK_STACK_SIZE (3 * 1024)
#define STATUS_LED_TASK_STACK_SIZE (2 * 1024)
#define WATCHDOG_TASK_STACK_SIZE (2 * 1024)

// RESET PIN
// TEYITLI: hoca "reset cip alive" dedi, eski vResetTask'in karsiligi.
// NOT: vResetTask dev'de KALDIRILDI (bkz. main.c) - reset pulse'i artik
// yazilim atmiyor, harici TPL5010 kendi periyoduyla cihazi resetliyor.
// Pin tanimi ve init'i dev'deki gibi yerinde birakildi.
#define RESET_PULSE_PIN 5
// vResetTask'in bekleme suresi (RTC senkronizasyonu + reset pulse arasi)
#define INTERVAL_MS 60000
// Harici TPL5010 watchdog'un cihazi resetlemesi beklenen, boot'tan itibaren gecen sure.
// TPL5010 direnci ~2 saate ayarli. Bu degerin biraz altinda kalmak guvenlidir cunku
// TPL5010 zamanlama toleransi nedeniyle reset nominal sureden bir miktar erken gelebilir.
#define ESTIMATE_RESET_MS (90u * 60u * 1000u) // ~1.5 saat (boot referansli, monotonik saatle olculur)

// THRESHOLD PIN + STATUS LED - ikisi de ayri, birlikte kullaniliyor
// (dev branch'te bu ikisi HARDWARE_VERSION'a gore birbirini disliyordu,
// ama bizim kartta hoca ikisini de ayri pin olarak verdi - bkz. CLAUDE.md
// "Kesinlesmis kararlar". Bu yuzden mutual-exclusion mantigi KALDIRILDI.)
// TEYITLI: hoca "role" dedi, eski Threshold pin'in karsiligi
#define THRESHOLD_PIN 6
// TEYITLI: "act led", gpio_test projesinde dogrulandi (Asama 2 Madde 1)
#define STATUS_LED_PIN 7

// ADC TANIMLARI
#define ADC_FIFO_SIZE 4000
// VRMS hesabi icin toplanan ornek sayisi (eski koddaki gibi)
#define VRMS_SAMPLE_SIZE 2000
#define SAMPLE_SIZE_PER_VRMS_CALC 200
// VRMS min/max/mean hesabi icin biriktirme tamponu boyutu
#define VRMS_BUFFER_SIZE 900
// TEYITLI: adc_vrms_test projesinde dogrulandi (Asama 2 Madde 5-6)
#define ADC_READ_PIN 3
// GPIO4 fiziksel olarak "bias/referans" pini olarak hocadan geldi, ama
// self-referencing RMS yontemine gecince yazilimda hic kullanilmiyordu -
// ADC_BIAS_PIN sabiti donanim/pin haritasi referansi olarak duruyor, ama
// bu pini okuyan kod (readBiasADCSample/BIAS_SAMPLE_SIZE) kullanicinin/
// hocanin istegiyle tamamen kaldirildi.
#define ADC_BIAS_PIN 4
// Sebeke frekansi - pencere boyutunu gercek olculen ornekleme hizina gore
// tam sayida periyoda oturtmak icin (bkz. adc.c/initADC()).
#define LINE_FREQ_HZ 50.0f
// Desimasyon (blok-ortalama) carpani - ekip arkadasinin adc_vrms_test'teki
// DECIMATION sabitiyle ayni (16). Gerekce: analog on-uctaki RC filtresi
// zaten dusuk bir frekansta kestigi icin ham ornekleme oversampling
// sayilir, 16'sar orneği ortalamak 50Hz sinyalini ihmal edilebilir
// duzeyde etkilerken ADC gurultusunu azaltir - gercek VRMS uretim
// hesabinda (calculateVRMSDecimated) kullaniliyor.
#define ADC_DECIMATION_FACTOR 16
// ADCSampleTask'in bir "burst"te (yield etmeden) art arda kac ornek
// aldigi - main.c ve adc.c'nin initADC() olcumu ayni degeri paylasiyor
// (bkz. ilgili yorumlar).
#define ADC_SAMPLE_BURST_SIZE 8
// ani degisim tespiti parametreleri
#define AMPLITUDE_THRESHOLD 5
// ESIK OLAY DEFINES
// bir pencerede toplanabilecek en fazla saniyelik VRMS ornegi (1 dk + pay)
#define TH_WINDOW_MAX_SAMPLES 72
// olayin baslamasi icin gereken ardisik pencere sayisi
#define TH_ENTER_WINDOWS 2
// olayin bitmesi icin gereken ardisik pencere sayisi (esik altinda)
#define TH_EXIT_WINDOWS 5
// Uzun olaylarda ara "suruyor" kaydi periyodu (dakika).
// TPL5010 donanim reseti ESTIMATE_RESET_MS (90 dk) periyodunda vurdugu icin bu
// deger ondan KUCUK olmali; aksi halde ara kayit hic yazilamadan reset gelir ve
// uzun bir ariza flash'ta hic iz birakmaz. Ayrica reset aninda kaybolan tepe
// degerin penceresi de bu kadardir.
#define TH_HEARTBEAT_MINUTES 30
// Acilista acik olayin baslangicini ararken halkada en fazla kac kayit geriye
// bakilacagi. Bulunamazsa son kaydin zamani baslangic sayilir.
#define TH_OPEN_EVENT_MAX_BACK 64
// Acik bir olay ancak son kaydi bu kadar dakikadan yeniyse devralinir. Donanim
// reseti 90 dk, ara kayit 30 dk; aradaki bosluk normalde 90 dk'yi asmaz. Daha
// eskiyse cihaz resetlenmemis, KAPALI kalmistir -- o araligi olaya saymak
// yaniltici olur, devralmayiz.
#define TH_RESUME_MAX_GAP_MINUTES 180
// histerezis: cikis esigi giris esiginin bu kadar altindadir (santivolt)
#define TH_HYSTERESIS_CV 50
// RTC'nin kurulmus sayilmasi icin en kucuk yil (2 haneli)
#define TH_RTC_MIN_VALID_YEAR 25
#define MEAN_CALCULATION_WINDOW_SIZE 20
#define MEAN_CALCULATION_SHIFT_SIZE 5

// RTC TANIMLARI
// TEYITLI: i2c_rtc_test projesinde dogrulandi (Asama 2 Madde 3) - 0x68 adresinde
// PT7C4338/DS1307 uyumlu cip bulundu
#define RTC_I2C_ADDRESS 0x68
#define RTC_REG_SECONDS 0x00
#define RTC_I2C_SDA_PIN 8
#define RTC_I2C_SCL_PIN 9

// OZEL KARAKTERLER (protokol cerceveleme - degismiyor)
#define SOH 0x01
#define STX 0x02
#define ETX 0x03
#define ACK 0x06
#define NACK 0x15
#define LINE_FEED '\n'
#define ETX_CHAR 0x03

#endif
