/*
 * BLE'nin okuyup yazdigi sayac verisinin GERCEK implementasyonu.
 *
 * ble_meter_test'teki meter_data_mock.c'nin yerini alir - artik sahte/dummy
 * degerler degil, meter_port'un gercek calisma-zamani durumunu (ADC/VRMS/
 * RTC/flash) okuyup/yaziyor. Alan seti, kartin GERCEKTEN urettigi kisa/uzun
 * okuma ciktisiyla (readout-mode.py -rms/-rml ile dogrulandi) VE ek "kart
 * durumu" bilgileriyle (uptime, bos bellek, ADC hizi, LED/gorev sagligi)
 * kuruldu.
 */
#include <stdlib.h>
#include "meter_data.h"
#include "common.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_partition.h"
#include "esp_system.h"

#include "header/project_globals.h"
#include "header/project_conf.h"
#include "header/mutex.h"
#include "header/spiflash.h"
#include "header/adc.h"
#include "header/defines.h"
#include "header/rtc.h"

#define NVS_NAMESPACE "meter_cfg"

// Runtime'da degistirilebilir kalibrasyon sabiti - dev'de/adc.c'de
// VRMS_MULTIPLICATION_VALUE sabit bir #define'di (148.8f), BLE'den
// yazilabilir olmasi icin globals.c'de gercek bir degisken olarak
// tanimlandi (bkz. project_globals.h), baslangic degeri hala ayni define.
extern float vrms_multiplication_value;

static char threshold_buf[16];
static char calibration_buf[16];
static char load_profile_buf[16];
// 24 -> 40: current_time alanlari int8_t/int16_t oldugu icin derleyici
// teorik en kotu durumu (-128 gibi 4 haneli) hesaba katip -Werror=
// format-truncation ile derlemeyi durduruyordu (gercekte deger araligi
// hep kucuk/pozitif, ama derleyici bunu bilmiyor) - ayni desen daha once
// spiflash.c/addSerialNumber()'da da gorulmustu.
static char rtc_time_buf[40];
static char baud_rate_buf[16] = "300";

// serial_number_buf KALDIRILDI - get_serial_number_str() artik dogrudan
// DEVICE_SERIAL_NUMBER makrosunu donduruyor, kopyalanacak bir sey yok.
static char firmware_version_buf[16];
static char production_date_buf[16];

static char vrms_max_buf[16] = "0.0";
static char vrms_min_buf[16] = "0.0";
static char vrms_mean_buf[16] = "0.0";
static char vrms_instant_buf[16] = "0.0";

// 22 slot (10 esik + 12 reset), her biri "T,10,26-08-04,19:09:45,011,08546;"
// gibi ~35 byte - rahat sigacak sekilde buyutuldu.
static char load_history_buf[1024] = "henuz okunmadi";

// ⚠️ BLE'de bir karakteristik degerinin ust siniri 512 BAYT (ATT spesifikasyonu).
// 10 esik kaydi (351 bayt) + 12 reset kaydi (267 bayt) = 618 bayt ediyor;
// fazlasi istemciye HIC ulasmiyor ve son kayit ORTADAN kesiliyordu - arayuzde
// "00 - undefined" olarak gorunen buydu.
//
// Karar: yarim kayit gondermektense AZ gondermek. Butceye TAM sigan kadar
// kayit yaziliyor (pratikte 7 reset kaydi), sigmayan hic yazilmiyor.
// RS485 tarafi (0.1.2*N) bu sinirdan etkilenmez - orada 12 kaydin tamami
// gelmeye devam eder.
#define BLE_HISTORY_MAX_BYTES 512
// En uzun reset kaydi: "R,12,00-01-01,01:44:32;" = 23 bayt
#define RS_HISTORY_ENTRY_MAX 23

static char uptime_buf[24];
static char free_heap_buf[24];
static char adc_rate_buf[48];
static char led_status_buf[32];

static void copy_bounded(char *dst, size_t dst_size, const uint8_t *src, uint16_t len)
{
    size_t n = len < dst_size - 1 ? len : dst_size - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

// NVS'e kalici yazma/okuma - basarisiz olursa (ilk acilis, namespace yok
// vs.) sessizce gecilir, RAM'deki varsayilan/mevcut deger kullanilmaya
// devam eder. (dev'in flash'inda kalibrasyon/baud rate icin ayri bir
// partition/slot yok - bu ikisi icin NVS kullanmak, threshold/load-profile
// gibi zaten flash'ta gercek yeri olanlardan farkli olarak, en pratik
// kalicilik yontemi.)
static void nvs_save_str(const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_open basarisiz (%d), '%s' kalici kaydedilemedi", err, key);
        return;
    }
    nvs_set_str(handle, key, value);
    nvs_commit(handle);
    nvs_close(handle);
}

static void nvs_load_str(const char *key, char *buf, size_t buf_size)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        return; // namespace henuz yok (ilk acilis) - varsayilan degerle devam
    }
    size_t required_size = buf_size;
    nvs_get_str(handle, key, buf, &required_size);
    nvs_close(handle);
}

// --- Meter Info: threshold, kalibrasyon, load profile periyodu, baud rate ---

const char *get_threshold_str(void)
{
    snprintf(threshold_buf, sizeof(threshold_buf), "%u", getVRMSThresholdValue());
    return threshold_buf;
}

void set_threshold_str(const uint8_t *data, uint16_t len)
{
    char tmp[16];
    copy_bounded(tmp, sizeof(tmp), data, len);
    int val = atoi(tmp);
    if (val < 0)
    {
        return;
    }
    // dev'in gercek yontemi: setVRMSThresholdValue() + updateThresholdSector()
    // (uart.c'deki setThresholdValue()'nun ayni ikilisi - flash'a kalici yaziyor)
    setVRMSThresholdValue((uint16_t)val);
    updateThresholdSector(th_sector_data);
    ESP_LOGI(TAG, "BLE: threshold guncellendi (kalici, flash): %d", val);
}

const char *get_calibration_str(void)
{
    snprintf(calibration_buf, sizeof(calibration_buf), "%.2f", vrms_multiplication_value);
    return calibration_buf;
}

void set_calibration_str(const uint8_t *data, uint16_t len)
{
    char tmp[16];
    copy_bounded(tmp, sizeof(tmp), data, len);
    float val = atof(tmp);
    if (val <= 0.0f)
    {
        return;
    }
    vrms_multiplication_value = val;
    nvs_save_str("calibration", tmp);
    ESP_LOGI(TAG, "BLE: kalibrasyon sabiti guncellendi (kalici, NVS): %.2f", val);
}

const char *get_load_profile_period_str(void)
{
    snprintf(load_profile_buf, sizeof(load_profile_buf), "%u", load_profile_record_period);
    return load_profile_buf;
}

void set_load_profile_period_str(const uint8_t *data, uint16_t len)
{
    char tmp[16];
    copy_bounded(tmp, sizeof(tmp), data, len);
    int val = atoi(tmp);
    if (val <= 0 || val > 255)
    {
        return;
    }
    load_profile_record_period = (uint8_t)val;
    // ⚠️ KARAR DEGISTI: dev'de bu deger flash'a kalici yazilmiyordu (RAM-only,
    // her aciliste varsayilana donuyordu) - ama kullanici BLE'den yapilan
    // degisikliklerin resetlenince KAYBOLMAMASINI istedi, bu yuzden BLE
    // yazma yolu icin NVS'e kalici kaydediliyor (dev'in kendi RS485/RP2040
    // davranisi degil, sadece bizim yeni BLE yazma yolumuz icin bir ekleme).
    nvs_save_str("loadprofile", tmp);
    ESP_LOGI(TAG, "BLE: load profile periyodu guncellendi (kalici, NVS): %d dakika", val);
}

// ⚠️ KARAR DEGISTI - artik tamamen salt okunur: gercek protokolde baud rate
// her istekte yeniden pazarlik ediliyor (exract_baud_rate_and_mode_from_message
// -> set_device_baud_rate), kalici/degistirilebilir bir "varsayilan baud"
// kavrami YOK. Yazilabilir birakmak, kullaniciya degistirdiginde gercekten
// bir seyin degistigi izlenimini (yanlislikla) veriyordu - kullanicinin
// kendisi bunu fark edip duzeltilmesini istedi. Artik sadece gercek
// baslangic/protokol degerini (BAUD_RATE define, defines.h) gosteriyor.
const char *get_baud_rate_str(void)
{
    snprintf(baud_rate_buf, sizeof(baud_rate_buf), "%d", BAUD_RATE);
    return baud_rate_buf;
}

void meter_data_load_from_nvs(void)
{
    char tmp[16];
    tmp[0] = '\0';
    nvs_load_str("calibration", tmp, sizeof(tmp));
    if (tmp[0] != '\0')
    {
        float val = atof(tmp);
        if (val > 0.0f)
        {
            vrms_multiplication_value = val;
        }
    }
    tmp[0] = '\0';
    nvs_load_str("loadprofile", tmp, sizeof(tmp));
    if (tmp[0] != '\0')
    {
        int val = atoi(tmp);
        if (val > 0 && val <= 255)
        {
            load_profile_record_period = (uint8_t)val;
        }
    }

    ESP_LOGI(TAG, "NVS'ten yuklendi: kalibrasyon=%.2f load_profile=%u dakika",
             vrms_multiplication_value, load_profile_record_period);
}

// --- Salt okunur alanlar ---

const char *get_rtc_time_str(void)
{
    snprintf(rtc_time_buf, sizeof(rtc_time_buf), "20%02d-%02d-%02d %02d:%02d:%02d",
              current_time.year, current_time.month, current_time.day,
              current_time.hour, current_time.min, current_time.sec);
    return rtc_time_buf;
}

// Sakamoto algoritmasi - verilen tarihin haftanin hangi gunu oldugunu
// hesaplar (0=Pazar, project_globals.h'deki dotw kuraliyla ayni).
// setTimePt7c4338() gun-ismi parametresini istiyor, RTC cipi kendisi
// hesaplamiyor.
static uint8_t compute_dotw(int year_full, int month, int day)
{
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    int y = year_full;
    if (month < 3)
    {
        y -= 1;
    }
    int dow = (y + y / 4 - y / 100 + y / 400 + t[month - 1] + day) % 7;
    return (uint8_t)dow;
}

// ⚠️ RTC saatini duzeltmek icin eklendi: gelistirme sirasinda RTC_SET_TEST_TIME
// ile rastgele bir test degeri (14:30:00) yazilmisti, gercek saatle hic
// eslesmiyordu - kalici cozum olarak RTC artik BLE'den yazilabilir, boyle
// bir daha koda saat gomup yeniden flaslamaya gerek kalmiyor (sahada
// teknisyen de ayni sekilde duzeltebilir).
void set_rtc_time_str(const uint8_t *data, uint16_t len)
{
    char tmp[32];
    copy_bounded(tmp, sizeof(tmp), data, len);

    int year, month, day, hour, min, sec;
    if (sscanf(tmp, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &min, &sec) != 6)
    {
        ESP_LOGE(TAG, "BLE: RTC saat formati gecersiz: '%s' (beklenen: YYYY-MM-DD HH:MM:SS)", tmp);
        return;
    }
    if (month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || min > 59 || sec > 59)
    {
        ESP_LOGE(TAG, "BLE: RTC saat degerleri gecersiz aralikta: '%s'", tmp);
        return;
    }

    uint8_t dotw = compute_dotw(year, month, day);
    uint8_t year2 = (uint8_t)(year % 100);

    if (setTimePt7c4338((uint8_t)sec, (uint8_t)min, (uint8_t)hour, dotw, (uint8_t)day, (uint8_t)month, year2))
    {
        // current_time'i hemen guncelle - WriteDebugTask zaten 1sn'de bir
        // senkronize ediyor ama telefon anlik geri okuyunca dogru gorsun.
        getTimePt7c4338(&current_time);
        ESP_LOGI(TAG, "BLE: RTC saati guncellendi: %s", tmp);
    }
    else
    {
        ESP_LOGE(TAG, "BLE: RTC saati yazilamadi (I2C hatasi?)");
    }
}

const char *get_serial_number_str(void)
{
    // Seri no artik flash'tan okunan RAM kopyasindan degil, dogrudan
    // DEVICE_SERIAL_NUMBER makrosundan geliyor (bkz. spiflash.c'deki not).
    // Makro derleme zamani sabiti oldugu icin ara bir tampona kopyalamaya
    // da gerek yok - dogrudan donduruluyor.
    return DEVICE_SERIAL_NUMBER;
}

const char *get_firmware_version_str(void)
{
    snprintf(firmware_version_buf, sizeof(firmware_version_buf), "%s", SOFTWARE_VERSION);
    return firmware_version_buf;
}

const char *get_production_date_str(void)
{
    snprintf(production_date_buf, sizeof(production_date_buf), "%s", PRODUCTION_DATE);
    return production_date_buf;
}

// --- Meter Live: VRMS max/min/mean (gercek cihazdaki 32.7.0/52.7.0/72.7.0 ile ayni kaynak) ---

const char *get_vrms_max_str(void) { return vrms_max_buf; }
const char *get_vrms_min_str(void) { return vrms_min_buf; }
const char *get_vrms_mean_str(void) { return vrms_mean_buf; }

void update_meter_live_data(void)
{
    if (xSemaphoreTake(xVRMSLastValuesMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        snprintf(vrms_max_buf, sizeof(vrms_max_buf), "%.1f", vrms_max_last);
        snprintf(vrms_min_buf, sizeof(vrms_min_buf), "%.1f", vrms_min_last);
        snprintf(vrms_mean_buf, sizeof(vrms_mean_buf), "%.1f", vrms_mean_last);
        xSemaphoreGive(xVRMSLastValuesMutex);
    }
    else
    {
        ESP_LOGE(TAG, "update_meter_live_data: xVRMSLastValuesMutex alinamadi");
    }
}

// YENI (kullanicinin istegiyle): vrms_instant, ADCReadTask'ta HER pencerede
// (mutex olmadan, tek yazan tek okuyan basit bir volatile float) guncelleniyor
// - burada sadece formatlayip metne ceviriyoruz, ekstra mutex gerekmiyor
// (bias_voltage gibi diger hafif "canli" degerlerle ayni desen).
const char *get_vrms_instant_str(void)
{
    snprintf(vrms_instant_buf, sizeof(vrms_instant_buf), "%.2f", vrms_instant);
    return vrms_instant_buf;
}

// --- Meter Control: kisa/uzun okuma + gecmis kayit ---

const char *get_load_history_str(void) { return load_history_buf; }

void trigger_short_read(void)
{
    // Gercek cihazda "kisa okuma" (-rms): sadece anlik durum ozeti - VRMS
    // ucluyu tazeliyoruz, digerleri (threshold/kalibrasyon/RTC/seri no vs.)
    // zaten getter'lar cagirildiginda taze okunuyor.
    update_meter_live_data();
    ESP_LOGI(TAG, "BLE kisa okuma tetiklendi: max=%s min=%s mean=%s",
             vrms_max_buf, vrms_min_buf, vrms_mean_buf);
}

// Esik asim (threshold_rec) ve reset (reset_dates) kayitlarini dogrudan
// flash'tan okuyup "T,slot,tarih,saat,vrms,sure;R,slot,tarih,saat" formatina
// cevirir - uart.c'deki send_threshold_records()/send_reset_dates() ile
// AYNI flash okuma mantigi (ayni partition'lar, ayni halka aritmetigi), ama
// RS485'e yazmak yerine bir metin tamponuna yaziyor.
//
// ⚠️ FORMAT DEGISTI (olay bazli kayitlara gecisle birlikte):
//   - vrms artik SANTIVOLT degil, "V.VV" seklinde ondalikli yaziliyor
//   - son alan varyans DEGIL, olayin DAKIKA cinsinden suresi
//     * 65535 = olay BASLADI, hala suruyor
//     * 65534 = olay SURUYOR (ara kayit), hala suruyor
//   Web arayuzu bu iki ozel degeri "devam ediyor" olarak gostermeli.
static void append_threshold_history(char *out, size_t out_size, size_t *pos)
{
    static uint8_t threshold_records_raw[FLASH_RECORD_SIZE * THRESHOLD_RECORD_OBIS_COUNT];

    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_THRESHOLD_REC);
    if (part == NULL)
    {
        ESP_LOGE(TAG, "threshold_rec partition bulunamadi!");
        return;
    }

    memset(threshold_records_raw, 0xFF, sizeof(threshold_records_raw));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        ESP_LOGE(TAG, "append_threshold_history: flash mutex alinamadi");
        return;
    }

    // Kayit alani bir HALKA tampon: alanin basindan degil, YAZMA KONUMUNDAN
    // geriye dogru yurunur. i = 0 en eski olacak sekilde dolduruyoruz ki
    // asagidaki artan slot numarasi kronolojik sirayla gitsin (RS485'teki
    // 96.77.4*N ile ayni kural: 1 = en eski, 10 = en yeni).
    uint16_t write_index = getThresholdWriteIndex();

    for (size_t i = 0; i < THRESHOLD_RECORD_OBIS_COUNT; i++)
    {
        uint16_t back = (uint16_t)(THRESHOLD_RECORD_OBIS_COUNT - i);
        uint16_t slot = thSlotBack(write_index, back, TH_RECORD_SLOT_COUNT);

        esp_partition_read(part, (size_t)slot * FLASH_RECORD_SIZE,
                           &threshold_records_raw[i * FLASH_RECORD_SIZE], FLASH_RECORD_SIZE);
    }

    xSemaphoreGive(xFlashMutex);

    for (size_t i = 0, idx = 1; i < THRESHOLD_RECORD_OBIS_COUNT; i++, idx++)
    {
        size_t offset = i * FLASH_RECORD_SIZE;
        int n;

        if (threshold_records_raw[offset] == 0xFF || threshold_records_raw[offset] == 0x00)
        {
            n = snprintf(out + *pos, out_size - *pos, "T,%d,00-00-00,00:00:00,000.00,00000;", (int)idx);
        }
        else
        {
            char year[3] = {(char)threshold_records_raw[offset], (char)threshold_records_raw[offset + 1], 0};
            char month[3] = {(char)threshold_records_raw[offset + 2], (char)threshold_records_raw[offset + 3], 0};
            char day[3] = {(char)threshold_records_raw[offset + 4], (char)threshold_records_raw[offset + 5], 0};
            char hour[3] = {(char)threshold_records_raw[offset + 6], (char)threshold_records_raw[offset + 7], 0};
            char min[3] = {(char)threshold_records_raw[offset + 8], (char)threshold_records_raw[offset + 9], 0};
            char sec[3] = {(char)threshold_records_raw[offset + 10], (char)threshold_records_raw[offset + 11], 0};
            uint16_t vrms = threshold_records_raw[offset + 13];
            vrms = (vrms << 8) + threshold_records_raw[offset + 12];
            uint16_t duration = threshold_records_raw[offset + 15];
            duration = (duration << 8) + threshold_records_raw[offset + 14];

            n = snprintf(out + *pos, out_size - *pos, "T,%d,%s-%s-%s,%s:%s:%s,%03d.%02d,%05d;",
                         (int)idx, year, month, day, hour, min, sec, vrms / 100, vrms % 100, duration);
        }

        if (n > 0 && (size_t)n < out_size - *pos)
        {
            *pos += (size_t)n;
        }
    }
}

static void append_reset_history(char *out, size_t out_size, size_t *pos)
{
    static uint8_t reset_dates_flash[FLASH_SECTOR_SIZE];
    static uint8_t reset_dates_raw[RESET_DATES_OBIS_COUNT * FLASH_RECORD_SIZE];

    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_RESET_DATES);
    if (part == NULL)
    {
        ESP_LOGE(TAG, "reset_dates partition bulunamadi!");
        return;
    }

    memset(reset_dates_raw, 0, sizeof(reset_dates_raw));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        ESP_LOGE(TAG, "append_reset_history: flash mutex alinamadi");
        return;
    }

    esp_partition_read(part, 0, reset_dates_flash, FLASH_SECTOR_SIZE);

    uint16_t idx = 0;
    while (idx < FLASH_SECTOR_SIZE)
    {
        if (reset_dates_flash[idx] == 0x00 || reset_dates_flash[idx] == 0xFF)
        {
            break;
        }
        idx += FLASH_RECORD_SIZE;
    }

    uint32_t end_offset = idx;
    uint32_t start_offset = (end_offset > FLASH_RECORD_SIZE * RESET_DATES_OBIS_COUNT)
                                 ? (end_offset - (FLASH_RECORD_SIZE * RESET_DATES_OBIS_COUNT))
                                 : 0;
    memcpy(reset_dates_raw, reset_dates_flash + start_offset, end_offset - start_offset);

    xSemaphoreGive(xFlashMutex);

    // Kalan bayt butcesine TAM sigan kayit sayisi. Sigmayanlar hic yazilmaz;
    // eskiden butce bitince son kayit ortadan kesiliyordu.
    //
    // ⚠️ Atlananlar BASTAKILER, yani EN ESKI kayitlar. Reset kayitlari
    // kronolojik siralidir (obis 1 = en eski, 12 = en yeni) ve pratikte
    // ilgilenilen sey son acilislardir - o yuzden yeniler korunup eskiler
    // dusuruluyor. Tersini istersen bu blogu silmen yeterli: o zaman
    // 1..7 (en eskiler) gonderilir.
    size_t budget = (BLE_HISTORY_MAX_BYTES > *pos) ? (BLE_HISTORY_MAX_BYTES - *pos) : 0;
    uint16_t fits = (uint16_t)(budget / RS_HISTORY_ENTRY_MAX);
    uint16_t skip = (fits < RESET_DATES_OBIS_COUNT) ? (uint16_t)(RESET_DATES_OBIS_COUNT - fits) : 0;

    if (skip > 0)
    {
        ESP_LOGW(TAG, "BLE 512 bayt siniri: en eski %u reset kaydi atlandi, %u kayit gonderiliyor "
                      "(RS485'te 12'sinin tamami gelmeye devam ediyor)",
                 (unsigned)skip, (unsigned)(RESET_DATES_OBIS_COUNT - skip));
    }

    for (uint16_t i = 0, obis = 1; i < sizeof(reset_dates_raw); i += FLASH_RECORD_SIZE, obis++)
    {
        int n;

        // Slot numarasi RS485'teki 0.1.2*N ile ayni kalsin diye obis yine de
        // ilerliyor; sadece yazma atlaniyor.
        if (obis <= skip)
        {
            continue;
        }

        if (reset_dates_raw[i] == 0xFF || reset_dates_raw[i] == 0x00)
        {
            n = snprintf(out + *pos, out_size - *pos, "R,%d,00-00-00,00:00:00;", obis);
        }
        else
        {
            char year[3] = {(char)reset_dates_raw[i], (char)reset_dates_raw[i + 1], 0};
            char month[3] = {(char)reset_dates_raw[i + 2], (char)reset_dates_raw[i + 3], 0};
            char day[3] = {(char)reset_dates_raw[i + 4], (char)reset_dates_raw[i + 5], 0};
            char hour[3] = {(char)reset_dates_raw[i + 6], (char)reset_dates_raw[i + 7], 0};
            char min[3] = {(char)reset_dates_raw[i + 8], (char)reset_dates_raw[i + 9], 0};
            char sec[3] = {(char)reset_dates_raw[i + 10], (char)reset_dates_raw[i + 11], 0};

            n = snprintf(out + *pos, out_size - *pos, "R,%d,%s-%s-%s,%s:%s:%s;",
                         obis, year, month, day, hour, min, sec);
        }

        if (n > 0 && (size_t)n < out_size - *pos)
        {
            *pos += (size_t)n;
        }
    }
}

void trigger_long_read(void)
{
    size_t pos = 0;
    load_history_buf[0] = '\0';
    append_threshold_history(load_history_buf, sizeof(load_history_buf), &pos);
    append_reset_history(load_history_buf, sizeof(load_history_buf), &pos);
    ESP_LOGI(TAG, "BLE uzun okuma tetiklendi (%d byte gecmis kayit)", (int)pos);
}

// --- YENI (kullanicinin istegiyle): gercek modem/okuyucu gibi tarih
// aralikli load profile sorgusu - RS485'teki "P.01(start;end)" mekanizmasinin
// (spiflash.c'deki getLoadProfileRecordsAsText/getLoadProfileAvailableDates,
// AYNI arama mantigini kullanir) BLE karsiligi. ---
static char load_profile_dates_buf[256];
static char load_profile_query_buf[2048];

const char *get_load_profile_dates_str(void)
{
    getLoadProfileAvailableDates(load_profile_dates_buf, sizeof(load_profile_dates_buf));
    return load_profile_dates_buf;
}

const char *get_load_profile_query_result_str(void)
{
    return load_profile_query_buf;
}

void trigger_load_profile_query(const uint8_t *data, uint16_t len)
{
    char tmp[32];
    copy_bounded(tmp, sizeof(tmp), data, len);

    int y1, m1, d1, y2, m2, d2;
    if (sscanf(tmp, "%d-%d-%d;%d-%d-%d", &y1, &m1, &d1, &y2, &m2, &d2) != 6)
    {
        snprintf(load_profile_query_buf, sizeof(load_profile_query_buf), "gecersiz tarih formati");
        ESP_LOGE(TAG, "BLE: load profile sorgusu gecersiz format: '%s' (beklenen: YY-MM-DD;YY-MM-DD)", tmp);
        return;
    }

    datetime_t dt_start = {0};
    datetime_t dt_end = {0};
    dt_start.year = (int16_t)y1;
    dt_start.month = (int8_t)m1;
    dt_start.day = (int8_t)d1;
    dt_start.hour = 0;
    dt_start.min = 0;
    dt_start.sec = 0;
    dt_end.year = (int16_t)y2;
    dt_end.month = (int8_t)m2;
    dt_end.day = (int8_t)d2;
    dt_end.hour = 23;
    dt_end.min = 59;
    dt_end.sec = 59;

    getLoadProfileRecordsAsText(&dt_start, &dt_end, load_profile_query_buf, sizeof(load_profile_query_buf));
    ESP_LOGI(TAG, "BLE: load profile sorgusu '%s' -> %d byte sonuc", tmp, (int)strlen(load_profile_query_buf));
}

// --- Meter Status (YENI, RS485/protokolde yok) ---

const char *get_uptime_str(void)
{
    int64_t uptime_sec = esp_timer_get_time() / 1000000;
    int hours = (int)(uptime_sec / 3600);
    int minutes = (int)((uptime_sec % 3600) / 60);
    int seconds = (int)(uptime_sec % 60);
    snprintf(uptime_buf, sizeof(uptime_buf), "%02d:%02d:%02d", hours, minutes, seconds);
    return uptime_buf;
}

const char *get_free_heap_str(void)
{
    snprintf(free_heap_buf, sizeof(free_heap_buf), "%lu KB", (unsigned long)(esp_get_free_heap_size() / 1024));
    return free_heap_buf;
}

const char *get_adc_rate_str(void)
{
    snprintf(adc_rate_buf, sizeof(adc_rate_buf), "%.0f Hz, pencere=%d ornek",
             getMeasuredSampleRateHz(), getWindowSampleCount());
    return adc_rate_buf;
}

const char *get_led_status_str(void)
{
    // current_pattern_id 0 = pattern_idle (hata yok). 1-11 arasi
    // LED_ERROR_CODE_* degerlerine karsilik geliyor (project_conf.h).
    static const char *names[] = {
        "NORMAL", "UART_OKUNAMIYOR", "MESAJ_ZAMANASIMI", "GECERSIZ_ISTEK_MODU",
        "GECERSIZ_SERI_NO", "FLASH_MUTEX_ALINAMADI", "FIFO_MUTEX_ALINAMADI",
        "VRMS_MUTEX_ALINAMADI", "ESIK_MUTEX_ALINAMADI", "ESIK_AYAR_MUTEX_ALINAMADI",
        "RX_TAMPON_TASMASI", "STACK_TASMASI"};
    int id = current_pattern_id;
    if (id < 0 || id >= (int)(sizeof(names) / sizeof(names[0])))
    {
        snprintf(led_status_buf, sizeof(led_status_buf), "BILINMEYEN(%d)", id);
    }
    else
    {
        snprintf(led_status_buf, sizeof(led_status_buf), "%s", names[id]);
    }
    return led_status_buf;
}

// --- Varsayilan ayarlara donme (YENI, kullanicinin istegiyle eklendi) ---
// Web sayfasindan "Varsayilan Ayarlara Sifirla" (onay istedikten sonra)
// gonderilen komutla cagriliyor - yazilabilir 4 alani (threshold,
// kalibrasyon, load profile periyodu, baud rate) fabrika degerlerine
// dondurur, hepsini kalici olarak (flash/NVS) yeniden kaydeder.
void reset_to_defaults(void)
{
    char tmp[16];

    setVRMSThresholdValue(5);
    updateThresholdSector(th_sector_data);

    vrms_multiplication_value = VRMS_MULTIPLICATION_VALUE;
    snprintf(tmp, sizeof(tmp), "%.2f", vrms_multiplication_value);
    nvs_save_str("calibration", tmp);

    load_profile_record_period = 15;
    nvs_save_str("loadprofile", "15");

    ESP_LOGI(TAG, "BLE: varsayilan ayarlara donuldu (threshold=5, kalibrasyon=%.2f, load_profile=15dk)",
             vrms_multiplication_value);
}

// --- Gecmis kayitlari silme (YENI, kullanicinin istegiyle eklendi) ---
// Web sayfasindaki "Sil" butonlarindan (onay istedikten sonra) gonderilen
// komutlarla cagriliyor.
void clear_threshold_history(void)
{
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_THRESHOLD_REC);
    if (part == NULL)
    {
        ESP_LOGE(TAG, "clear_threshold_history: threshold_rec partition bulunamadi");
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        ESP_LOGE(TAG, "clear_threshold_history: flash mutex alinamadi");
        return;
    }
    esp_partition_erase_range(part, 0, part->size);
    xSemaphoreGive(xFlashMutex);

    // th_flash_buf (RAM'deki sektor kopyasi) da temizlenmeli - aksi halde
    // bir sonraki writeThresholdRecord() flash'i yeni sildigimizi bilmeyip
    // eski RAM icerigini geri yazabilirdi (bkz. adc.c'deki ayni uyari).
    th_sector_data = 0;
    updateThresholdSector(0);
    memset(th_flash_buf, 0, FLASH_SECTOR_SIZE);

    ESP_LOGI(TAG, "BLE: esik asim gecmisi silindi");
}

void clear_reset_history(void)
{
    const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_RESET_DATES);
    if (part == NULL)
    {
        ESP_LOGE(TAG, "clear_reset_history: reset_dates partition bulunamadi");
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        ESP_LOGE(TAG, "clear_reset_history: flash mutex alinamadi");
        return;
    }
    esp_partition_erase_range(part, 0, part->size);
    xSemaphoreGive(xFlashMutex);

    ESP_LOGI(TAG, "BLE: reset/acilis gecmisi silindi");
}

// --- Bos bellek degistikce bildirim (YENI, kullanicinin istegiyle eklendi) ---
// gatt_svc.c'deki send_status_indication() tarafindan periyodik olarak
// cagriliyor - sadece deger GERCEKTEN degismisse true doner, boylece
// gereksiz BLE trafigi/bildirimi olmuyor.
bool free_heap_changed_since_last_check(void)
{
    static uint32_t last_heap = 0;
    uint32_t current = esp_get_free_heap_size();
    if (current != last_heap)
    {
        last_heap = current;
        return true;
    }
    return false;
}
