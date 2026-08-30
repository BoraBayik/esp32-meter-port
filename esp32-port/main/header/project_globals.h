#ifndef PROJECT_GLOBALS_H
#define PROJECT_GLOBALS_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "header/defines.h"
#include "header/project_conf.h"

// ============================================================================
// Bu dosya `dev` branch'teki blink/header/project_globals.h dosyasindan
// ESP32-C3'e uyarlanmistir.
//
// ONEMLI FARK: dev branch "pico/util/datetime.h" (Pico SDK) icindeki
// datetime_t tipini kullaniyordu - bu tip ESP-IDF'te yok. Asagida kendi
// esdeger struct'imizi tanimliyoruz (rtc.h portlanirken kullanilacak).
// ============================================================================

typedef struct
{
    int16_t year;
    int8_t month;
    int8_t day;
    int8_t dotw; // day of the week, 0 = Pazar
    int8_t hour;
    int8_t min;
    int8_t sec;
} datetime_t;

extern volatile int current_pattern_id;
extern volatile bool play_once;

typedef struct
{
    uint16_t data[ADC_FIFO_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} ADC_FIFO;

typedef struct
{
    uint8_t vrms_max;
    uint8_t vrms_min;
    uint8_t vrms_mean;
    uint8_t vrms_max_dec;
    uint8_t vrms_min_dec;
    uint8_t vrms_mean_dec;
} VRMS_VALUES_RECORD;

enum ListeningStates
{
    BCCError = 0,
    DataError,
    Password,
    Reading,
    TimeSet,
    DateSet,
    ProductionInfo,
    SetThreshold,
    GetThreshold,
    ThresholdPin,
    GetSuddenAmplitudeChange,
    ReadTime,
    ReadDate,
    ReadSerialNumber,
    ReadLastVRMSMax,
    ReadLastVRMSMin,
    ReadLastVRMSMean,
    ReadResetDates,
    GetThresholdObis,
    BreakMessage
};

struct FlashData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    uint8_t max_volt;
    uint8_t max_volt_dec;
    uint8_t min_volt;
    uint8_t min_volt_dec;
    uint8_t mean_volt;
    uint8_t mean_volt_dec;
};

// Esik kaydi artik OLAY bazlidir, saniyelik ornek bazli degil.
//   - Olay baslarken: zaman damgasi = olayin basi, vrms = tetikleyen deger,
//     duration = TH_DURATION_STARTED (0xFFFF).
//   - Olay biterken:  zaman damgasi = olayin sonu, vrms = olay boyunca gorulen
//     TEPE deger, duration = dakika cinsinden sure.
// vrms artik SANTIVOLT'tur (573 = 5.73 V); eskiden volt cinsinden tamsayiya
// kirpiliyor ve ondalik tamamen kayboluyordu.
// Boyut 16 baytta sabit kalir; flash yerlesimi degismez.
struct ThresholdData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint16_t vrms;     // santivolt
    uint16_t duration; // dakika, 0xFFFF = olay suruyor
};

struct AmplitudeChangeData
{
    char year[2];
    char month[2];
    char day[2];
    char hour[2];
    char min[2];
    char sec[2];
    uint8_t sample_buffer[ADC_FIFO_SIZE];
    float vrms_values_buffer[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t variance;
    uint8_t padding[42];
};

struct AmplitudeChangeTimerCallbackParameters
{
    float vrms_values_buffer[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    uint16_t variance;
    size_t adc_fifo_size;
    size_t vrms_values_buffer_size_bytes;
};

typedef struct
{
    const uint16_t *sequence;
    uint16_t length;
} LedPattern;

// ADC DEGISKENLERI
extern ADC_FIFO adc_fifo;
extern uint8_t load_profile_record_period;
extern volatile float vrms_max_last;
extern volatile float vrms_min_last;
extern volatile float vrms_mean_last;
// YENI (kullanicinin istegiyle, RS485/protokolde karsiligi yok, sadece
// BLE'ye ozel): ADCReadTask'in her pencerede (ortalama saniyede birkac
// kere) hesapladigi HAM/anlik VRMS degeri - vrms_max/min/mean_last'in
// aksine bu periyodik degil, her hesaplamada guncelleniyor.
extern volatile float vrms_instant;
extern uint16_t vrms_threshold;
extern uint8_t threshold_set_before;
// Calisma-zamaninda degistirilebilir kalibrasyon sabiti (BLE'den yazilabilir
// olmasi icin) - dev'de/ilk portta VRMS_MULTIPLICATION_VALUE sabit bir
// #define'di (148.8f), artik gercek bir degisken (baslangic degeri hala
// ayni define'dan geliyor, bkz. globals.c).
extern float vrms_multiplication_value;

// UART DEGISKENLERI
extern bool password_correct_flag;

// FLASH (SPI) DEGISKENLERI
// ⚠️ serial_number[] KALDIRILDI: seri no artik flash'tan okunan bir RAM
// kopyasi degil, dogrudan DEVICE_SERIAL_NUMBER makrosu (project_conf.h).
extern uint16_t sector_data;
extern uint16_t th_sector_data;
extern struct FlashData flash_data[FLASH_SECTOR_SIZE / sizeof(struct FlashData)];
extern struct ThresholdData th_flash_buf[FLASH_SECTOR_SIZE / sizeof(struct ThresholdData)];

// RTC DEGISKENLERI
extern char datetime_buffer[64];
extern char *datetime_str;
extern datetime_t current_time;

// FreeRTOS TASK HANDLE'LARI
extern TaskHandle_t xADCHandle;
extern TaskHandle_t xADCSampleHandle;
extern TaskHandle_t xUARTHandle;
extern TaskHandle_t xResetHandle;
extern TaskHandle_t xGetRTCHandle;
extern TaskHandle_t xStatusLedHandle;
extern TaskHandle_t xWatchdogHandle;

extern SemaphoreHandle_t xFlashMutex;
extern SemaphoreHandle_t xFIFOMutex;
extern SemaphoreHandle_t xVRMSLastValuesMutex;
extern SemaphoreHandle_t xVRMSThresholdMutex;
extern SemaphoreHandle_t xThresholdSetFlagMutex;

extern const uint16_t pattern_idle[];

// LED Hata Desenleri
extern const uint16_t led_pattern_uart_not_readable[];
extern const uint16_t led_pattern_message_timeout[];
extern const uint16_t led_pattern_invalid_request_mode[];
extern const uint16_t led_pattern_invalid_serial_number[];
extern const uint16_t led_pattern_flash_mutex_not_taken[];
extern const uint16_t led_pattern_fifo_mutex_not_taken[];
extern const uint16_t led_pattern_vrms_values_mutex_not_taken[];
extern const uint16_t led_pattern_vrms_threshold_mutex_not_taken[];
extern const uint16_t led_pattern_threshold_set_mutex_not_taken[];
extern const uint16_t led_pattern_rx_buffer_overflow_isr[];
extern const LedPattern patterns[];

#endif
