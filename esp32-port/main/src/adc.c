#include <string.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_partition.h"

#include "header/project_globals.h"
#include "header/print.h"
#include "header/spiflash.h"
#include "header/mutex.h"
#include "header/adc.h"
#include "header/bcc.h"
#include "header/threshold_event.h"

// Halka tampon aritmetigi kayit boyutunun 16 bayt kalmasina dayaniyor.
// Okuma tarafi (uart.c) alanlari SABIT OFFSET ile cozuyor; derleyici araya
// dolgu koyarsa flash yerlesimi sessizce bozulur. O yuzden burada zorluyoruz.
_Static_assert(sizeof(struct ThresholdData) == FLASH_RECORD_SIZE,
               "ThresholdData FLASH_RECORD_SIZE ile ayni boyutta olmali");
_Static_assert(offsetof(struct ThresholdData, vrms) == 12,
               "vrms alani 12. bayttan baslamali (uart.c sabit offset kullaniyor)");
_Static_assert(offsetof(struct ThresholdData, duration) == 14,
               "duration alani 14. bayttan baslamali (uart.c sabit offset kullaniyor)");

// dev branch'teki blink/src/adc.c dosyasindan portlanmistir. Asil
// degisiklikler calculateVRMS() ve ADC donanim erisiminde - bkz. adc.h
// basindaki aciklama. calculateVariance/getMean/calculateVRMSValuesPerSecond/
// writeThresholdRecord/detectSuddenAmplitudeChangeWithDerivative/
// setAmplitudeChangeParameters mantik olarak dev ile BIREBIR AYNI.

static const char *TAG = "adc";

#define ASSUMED_REFERENCE_VOLTAGE 3.3f
#define ADC_FULL_SCALE_COUNT 4095.0f
#define ASSUMED_REFERENCE_VOLTAGE_PER_COUNT_FALLBACK (3.3f / 4095.0f)

static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_channel_t s_main_channel;
static adc_cali_handle_t s_main_cali = NULL;

// Kalibre egim (V/sayim) - initADC() sirasinda BIR KERE hesaplanip
// onbelleklenir (ekip arkadasinin optimizasyonu: her ornekte kalibrasyon
// cagirmak yerine, pencere/olcum basina bir kere egim cikarilir).
static float s_main_volts_per_count = ASSUMED_REFERENCE_VOLTAGE_PER_COUNT_FALLBACK;

// Pencere boyutu (ornek sayisi) - dev'deki gibi sabit VRMS_SAMPLE_SIZE
// DEGIL, initADC() sirasinda GERCEK olculen ornekleme hizina gore, tam
// sayida 50Hz periyoduna oturacak sekilde hesaplaniyor (ekip arkadasinin
// adc_vrms_test'teki yontemi - "yarim dalga kesilmesi" sistematik hatasini
// onluyor). VRMS_SAMPLE_SIZE'i asmayacak sekilde sinirlandirilir (mevcut
// statik tamponlarin boyutunu asmamak icin).
static int s_window_samples = VRMS_SAMPLE_SIZE;
// initADC()'te olculen gercek hiz (Hz) - main.c'nin ADCReadTask'inda
// "beklenen sure" hesaplayip gercek gecen sureyle karsilastirmasi (sapma
// kontrolu) icin disariya aciliyor (getMeasuredSampleRateHz()).
static float s_measured_rate_hz = 0.0f;

static void setup_adc_channel(int gpio, adc_channel_t *out_channel, adc_cali_handle_t *out_cali)
{
    adc_unit_t unit;
    ESP_ERROR_CHECK(adc_oneshot_io_to_channel(gpio, &unit, out_channel));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, *out_channel, &chan_config));

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = *out_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_config, out_cali);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO%d icin kalibrasyon semasi kurulamadi: %s", gpio, esp_err_to_name(err));
        *out_cali = NULL;
    }
}

// AC olcumu icin kalibrasyonun sadece EGIMI lazim (ofset, fark alinirken
// zaten sadelesiyor). Iki noktadan egimi bir kez cikarir - ekip arkadasinin
// cali_volts_per_count() fonksiyonunun birebir portu.
static float cali_volts_per_count(adc_cali_handle_t cali)
{
    if (cali == NULL)
        return ASSUMED_REFERENCE_VOLTAGE / ADC_FULL_SCALE_COUNT;
    int lo = 0, hi = 0;
    adc_cali_raw_to_voltage(cali, 400, &lo);
    adc_cali_raw_to_voltage(cali, 3600, &hi);
    if (hi <= lo)
        return ASSUMED_REFERENCE_VOLTAGE / ADC_FULL_SCALE_COUNT;
    return ((float)(hi - lo) / 1000.0f) / 3200.0f;
}

uint8_t initADC(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_config, &s_adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "ADC birimi olusturulamadi: %s", esp_err_to_name(err));
        return 0;
    }

    setup_adc_channel(ADC_READ_PIN, &s_main_channel, &s_main_cali);
    // ⚠️ ADC_BIAS_PIN (GPIO4) kanal kurulumu KALDIRILDI - self-referencing
    // RMS'de hic kullanilmiyordu, kullanicinin/hocanin istegiyle cikarildi.

    s_main_volts_per_count = cali_volts_per_count(s_main_cali);

    // Gercek ornekleme hizini olc (ekip arkadasinin measure_sample_rate()
    // fonksiyonunun portu) - 1000 ornek okuyup gecen gercek sureden hizi
    // hesaplar. Bu, ADCSampleTask henuz baslamadan, tek seferlik yapiliyor.
    //
    // ⚠️ ONEMLI DUZELTME: ilk taslakta bu olcum KESINTISIZ (yield'siz) bir
    // dongu ile yapiliyordu - ama gercek ADCSampleTask, CPU'yu diger
    // gorevlere birakmak icin her ADC_SAMPLE_BURST_SIZE ornekte bir kisa
    // bir vTaskDelay(1) yapiyor (bkz. main.c). Kesintisiz olcum, bu
    // bekleme payini hesaba katmadigi icin GERCEK calisma hizindan daha
    // YUKSEK bir sonuc verirdi - tam onlemeye calistigimiz sistematik
    // sapmayi (yanlis pencere boyutu) geri getirirdi. Duzeltme: olcum de
    // AYNI burst+yield deseniyle yapiliyor, boylece gercek calisma zamani
    // hizini dogru yansitiyor.
    const int total_measure_samples = 1000;
    int64_t t0 = esp_timer_get_time();
    int measured = 0;
    while (measured < total_measure_samples)
    {
        for (int i = 0; i < ADC_SAMPLE_BURST_SIZE && measured < total_measure_samples; i++)
        {
            int raw = 0;
            adc_oneshot_read(s_adc_handle, s_main_channel, &raw);
            measured++;
        }
        vTaskDelay(1);
    }
    int64_t dt = esp_timer_get_time() - t0;
    float measured_rate_hz = (dt > 0) ? ((float)total_measure_samples / ((float)dt / 1000000.0f)) : 0.0f;

    if (measured_rate_hz > 0.0f)
    {
        // Pencereyi, VRMS_SAMPLE_SIZE'i asmayacak sekilde, 50Hz'in en
        // buyuk tam kati kadar ornek icerecek sekilde ayarla - boylece
        // mevcut sabit boyutlu tamponlar (adc_samples_buffer vs.) hicbir
        // degisiklik gerektirmeden guvenle kullanilabiliyor.
        float samples_per_period = measured_rate_hz / LINE_FREQ_HZ;
        int max_periods = (samples_per_period > 0.0f) ? (int)(VRMS_SAMPLE_SIZE / samples_per_period) : 0;
        if (max_periods < 1)
        {
            max_periods = 1;
        }
        int computed = (int)(max_periods * samples_per_period + 0.5f);
        if (computed > VRMS_SAMPLE_SIZE)
        {
            computed = VRMS_SAMPLE_SIZE;
        }
        if (computed < ADC_DECIMATION_FACTOR)
        {
            computed = ADC_DECIMATION_FACTOR;
        }
        s_window_samples = computed;
    }
    s_measured_rate_hz = measured_rate_hz;

    ESP_LOGI(TAG, "initADC tamamlandi. ana kanal V/sayim=%.6f (ham=%.6f)",
             s_main_volts_per_count, ASSUMED_REFERENCE_VOLTAGE / ADC_FULL_SCALE_COUNT);
    ESP_LOGI(TAG, "Olculen ornekleme hizi: ~%.0f Hz -> pencere %d ornek (VRMS_SAMPLE_SIZE=%d)",
             measured_rate_hz, s_window_samples, VRMS_SAMPLE_SIZE);

    return 1;
}

int getWindowSampleCount(void)
{
    return s_window_samples;
}

float getMeasuredSampleRateHz(void)
{
    return s_measured_rate_hz;
}

uint16_t readMainADCSample(void)
{
    int raw = 0;
    adc_oneshot_read(s_adc_handle, s_main_channel, &raw);
    return (uint16_t)raw;
}

float getMainVoltsPerCount(void)
{
    return s_main_volts_per_count;
}

int decimateSamples(const uint16_t *in, int n, int factor, float *out)
{
    int m = n / factor;
    for (int i = 0; i < m; i++)
    {
        double s = 0.0;
        for (int j = 0; j < factor; j++)
            s += (double)in[i * factor + j];
        out[i] = (float)(s / factor);
    }
    return m;
}

uint16_t calculateVariance(uint16_t *buffer, uint16_t size)
{
    uint64_t total = 0;
    uint64_t mean;
    uint64_t variance_total = 0;

    for (uint16_t i = 0; i < size; i++)
    {
        total += buffer[i];
    }

    mean = total / size;

    for (uint16_t i = 0; i < size; i++)
    {
        int32_t mult = buffer[i] - mean;
        variance_total += mult * mult;
    }

    return (uint16_t)(variance_total / (size - 1));
}

// ⚠️ ASIL DUZELTME (bkz. adc.h basindaki aciklama): DC bileseni, ayri bir ADC
// kanalindan (bias_voltage parametresi) DEGIL, bufferin KENDI ortalamasindan
// cikariliyor (self-referencing). bias_voltage parametresi imza uyumlulugu
// icin duruyor ama KULLANILMIYOR.
float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage)
{
    (void)bias_voltage; // artik capraz-kanal referans olarak kullanilmiyor

    double sum = 0.0;
    for (size_t i = 0; i < size; i++)
        sum += (double)buffer[i];
    double mean = sum / (double)size;

    double acc = 0.0;
    for (size_t i = 0; i < size; i++)
    {
        double d = (double)buffer[i] - mean;
        acc += d * d;
    }
    float rms_counts = (float)sqrt(acc / (double)size);

    return rms_counts * s_main_volts_per_count * vrms_multiplication_value;
}

// ⚠️ EKIP ARKADASININ adc_vrms_test'te ozellikle isaret ettigi, GERCEK URETIM
// YONTEMI olarak benimsenen hesap: desimasyonlu (blok-ortalamali) self-
// referencing RMS. Gerekce (ekip arkadasinin kod yorumundan): analog on-uctaki
// RC filtresi zaten ~236 Hz'de kesiyor, yani ham ornekleme bu frekansa gore
// asiri-orneklemedir (oversampling) - `decimation_factor` kadar orneği
// bloklar halinde ortalamak 50 Hz sinyalini ihmal edilebilir duzeyde (~%0.25)
// zayiflatirken, ADC gurultusunu yaklasik sqrt(decimation_factor) kat azaltir.
// `calculateVRMS()` (ham, decimasyonsuz) hala duruyor - rapor/karsilastirma
// icin, ama gercek uretim kodu (main.c/ADCReadTask) artik bunu cagiriyor.
float calculateVRMSDecimated(uint16_t *buffer, size_t size, int decimation_factor)
{
    static float decim_buf[VRMS_SAMPLE_SIZE];
    int m = decimateSamples(buffer, (int)size, decimation_factor, decim_buf);
    if (m <= 1)
    {
        return calculateVRMS(buffer, size, 0.0f);
    }

    double sum = 0.0;
    for (int i = 0; i < m; i++)
        sum += (double)decim_buf[i];
    double mean = sum / (double)m;

    double acc = 0.0;
    for (int i = 0; i < m; i++)
    {
        double d = (double)decim_buf[i] - mean;
        acc += d * d;
    }
    float rms_counts = (float)sqrt(acc / (double)m);

    return rms_counts * s_main_volts_per_count * vrms_multiplication_value;
}

float getMean(uint16_t *buffer, size_t size)
{
    float total = 0;

    for (size_t i = 0; i < size; i++)
        total += (float)buffer[i];

    if (size == 0)
    {
        return 0;
    }
    else
    {
        return (total / size);
    }
}

// Aktif esik sektorunun ham icerigini flash'tan okur. dev'de bu, XIP uzerinden
// dogrudan pointer erisimiydi; ESP'de esp_partition API'si gerekiyor.
// ⚠️ Mutex ALMAZ - cagiran taraf sorumludur. uart.c bu fonksiyonu zaten kendi
// mutex blogunun icinden cagiriyor; burada tekrar almak (rekursif olmayan)
// mutex'te kilitlenmeye yol acardi.
static bool readThresholdSector(uint16_t sector, uint8_t *out)
{
    if (out == NULL || sector >= TH_RECORD_SECTOR_COUNT)
    {
        return false;
    }

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_THRESHOLD_REC);

    if (part == NULL)
    {
        return false;
    }

    return esp_partition_read(part, (size_t)sector * FLASH_SECTOR_SIZE, out, FLASH_SECTOR_SIZE) == ESP_OK;
}

// Halkadaki bir sonraki yazma konumunu (mutlak slot indeksi) hesaplar.
uint16_t getThresholdWriteIndex(void)
{
    // 4 KB - stack'e sigmaz, static olmali (UARTTask'in yigini 4 KB).
    static uint8_t sector_buf[FLASH_SECTOR_SIZE];
    uint16_t offset = FLASH_SECTOR_SIZE;

    if (readThresholdSector(th_sector_data, sector_buf))
    {
        offset = thFindFreeOffset(sector_buf, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE);
    }

    th_write_pos_t pos = thNextWritePos(th_sector_data, offset, FLASH_SECTOR_SIZE,
                                        FLASH_RECORD_SIZE, TH_RECORD_SECTOR_COUNT);

    return (uint16_t)(((uint32_t)pos.sector * TH_RECORDS_PER_SECTOR) + pos.slot_in_sector);
}

// thFindOpenEvent icin geri cagirim: halkada `back` kayit geriye giden ham
// 16 bayti okur (back = 1 en son yazilan kayit).
static bool fetchThresholdRecordBack(void *ctx, uint16_t back, uint8_t *out)
{
    if (ctx == NULL || out == NULL || back == 0 || back > TH_RECORD_SLOT_COUNT)
    {
        return false;
    }

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_THRESHOLD_REC);

    if (part == NULL)
    {
        return false;
    }

    uint16_t write_index = *(const uint16_t *)ctx;
    uint16_t slot = thSlotBack(write_index, back, TH_RECORD_SLOT_COUNT);

    return esp_partition_read(part, (size_t)slot * FLASH_RECORD_SIZE, out, FLASH_RECORD_SIZE) == ESP_OK;
}

// Acilista cagrilir: onceki calisma bir olayin ortasinda kesildiyse (son kayit
// hala "acik" isaretliyse) o olayin baslangicini, tepesini ve son kayit anini
// dondurur.
bool findOpenThresholdEvent(th_time_t *start_time, uint16_t *peak_cv, th_time_t *last_record_time)
{
    uint16_t write_index = getThresholdWriteIndex();

    return thFindOpenEvent(fetchThresholdRecordBack, &write_index, TH_OPEN_EVENT_MAX_BACK,
                           start_time, peak_cv, last_record_time);
}

// Write threshold data to flash.
// Kayit alani bir HALKA tampondur: son sektor dolunca 0. sektore donulur, o
// sektor silinip uzerine yazilmaya devam edilir (en eski kayitlar duser).
//
// NOT: th_flash_buf artik her yazmadan once flash'tan GERI OKUNUYOR. Eskiden
// RAM'de sifir basliyordu ve her reset sonrasi ilk kayit sektoru sifirdan
// yaziyormus gibi davranip o sektordeki onceki kayitlari siliyordu.
void writeThresholdRecord(const struct ThresholdData *record)
{
    PRINTF("writing threshold record\r\n");

    if (record == NULL)
    {
        return;
    }

    // Son savunma hatti: bu fonksiyon flash SILIYOR. th_sector_data bir sekilde
    // bozulursa silme kendi alanimizin disina duser. Yazmaktansa yazmamak yeglenir.
    if (th_sector_data >= TH_RECORD_SECTOR_COUNT)
    {
        PRINTF("WRITETHRESHOLDRECORD: sektor numarasi araligin disinda (%d), kayit iptal\r\n", th_sector_data);
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex received\r\n");
        readThresholdSector(th_sector_data, (uint8_t *)th_flash_buf);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("WRITETHRESHOLDRECORD: memcpy mutex error\r\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }

    // Kaydin alanlari cagiran tarafta (olay mantiginda) doldurulmustur; burada
    // sadece halkadaki yerine konur.
    uint16_t free_offset = thFindFreeOffset((const uint8_t *)th_flash_buf, FLASH_SECTOR_SIZE, FLASH_RECORD_SIZE);
    th_write_pos_t pos = thNextWritePos(th_sector_data, free_offset, FLASH_SECTOR_SIZE,
                                        FLASH_RECORD_SIZE, TH_RECORD_SECTOR_COUNT);

    if (pos.sector_changed)
    {
        PRINTF("WRITETHRESHOLDRECORD: sector %d dolu, %d. sektore geciliyor\r\n", th_sector_data, pos.sector);

        th_sector_data = pos.sector;

        // Yeni sektorun RAM kopyasini silinmis hale (0xFF) getir. Bu sektor
        // asagida silinip bastan yazildigi icin icindeki en eski kayitlar duser
        // -- halkanin basa donmesi tam olarak budur.
        memset(th_flash_buf, 0xFF, FLASH_SECTOR_SIZE);
        updateThresholdSector(th_sector_data);
    }

    th_flash_buf[pos.slot_in_sector] = *record;

    PRINTF("WRITETHRESHOLDRECORD: kayit sektor %d slot %d/%d konumuna yazildi\r\n",
           th_sector_data, pos.slot_in_sector, (int)TH_RECORDS_PER_SECTOR);

    // th_flash_buf'in TAMAMINI (bir sektor) ilgili sektore yaz - dev'deki
    // gibi bu adim HER ZAMAN calisir (normal ekleme VE sektor degisimi
    // durumlarinin ikisinde de), if/else disinda.
    //
    // ⚠️ GERCEK BIR HATA BURADAYDI, BULUNUP DUZELTILDI: writeThresholdRecordsSectorToFlash()
    // kendi icinde ZATEN xFlashMutex aliyor (spiflash.c). Burada AYRICA disaridan
    // bir kez daha xFlashMutex alip, onu ELDE TUTARKEN o fonksiyonu cagirmak,
    // ayni gorevin ayni (recursive olmayan) mutex'i ikinci kez almaya calismasina
    // sebep oluyordu - bu HER ZAMAN (baska hicbir seyle cakismasa bile) 250ms
    // bekleyip basarisiz oluyordu, ve "threshold record written to flash." mesaji
    // kosulsuz basildigi icin gercekte hicbir zaman flash'a yazilmamis kayitlari
    // "basarili" gibi gosteriyordu. Duzeltme: disaridaki fazladan kilit sarmalayicisi
    // kaldirildi, fonksiyon dogrudan cagriliyor - kendi mutex'ini kendi yonetiyor.
    writeThresholdRecordsSectorToFlash(th_sector_data);
}

#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size)
{
    for (uint16_t i = 1; i < buffer_size; i++)
    {
        float derivative = sample_buf[i] - sample_buf[i - 1];

        if (fabsf(derivative) > AMPLITUDE_THRESHOLD)
        {
            PRINTF("Sudden amplitude change detected at index %d: %f\r\n", i, fabsf(derivative));
            return 1;
        }
    }

    return 0;
}
#endif

void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage)
{
    for (uint16_t i = 0; i < buffer_size; i += sample_size_per_vrms_calc)
    {
        float vrms = calculateVRMS(sample_buf + i, sample_size_per_vrms_calc, bias_voltage);
        vrms_buffer[i / sample_size_per_vrms_calc] = vrms;
    }

    // GECICI TANI SUSTURMA: bu satir her pencerede (saniyede birkac kere)
    // 9 sayilik bir liste basip ekrani kalabalik ediyordu, okunabilirlik
    // icin susturuldu - hesaplama (vrms_buffer'in doldurulmasi) aynen
    // devam ediyor, sadece ekrana yazma kismi kapatildi.
    // PRINTF("VRMS VALUES PER SECOND:");
    // printBufferFloat(vrms_buffer, buffer_size / sample_size_per_vrms_calc);
}

void setAmplitudeChangeParameters(struct AmplitudeChangeTimerCallbackParameters *ac_data, float *vrms_values_buffer, uint16_t variance, size_t adc_fifo_size, size_t vrms_values_buffer_size_bytes)
{
    memcpy(ac_data->vrms_values_buffer, vrms_values_buffer, vrms_values_buffer_size_bytes);
    ac_data->vrms_values_buffer_size_bytes = vrms_values_buffer_size_bytes;
    ac_data->variance = variance;
    ac_data->adc_fifo_size = adc_fifo_size;
}
