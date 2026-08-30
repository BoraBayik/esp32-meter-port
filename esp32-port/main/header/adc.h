#ifndef ADC_H
#define ADC_H

#include <stdint.h>
#include <stddef.h>
#include "header/project_globals.h"

// dev branch'teki blink/header/adc.h'den ESP32-C3'e uyarlanmistir.
//
// ⚠️ ONEMLI FARK - "bias referans yontemi" karari (bkz. CLAUDE.md Asama 2
// Madde 5-6): dev/RP2040 kodu calculateVRMS() icinde her ornekten AYRI BIR
// ADC kanalinin (bias/GPIO4) ortalamasini cikariyordu (capraz-kanal referans).
// Ekip arkadasinin adc_vrms_test'te fiziksel olarak test edip dogruladigi ve
// "buna gore yaz" dedigi yontem bunun yerine her orneği KENDI penceresinin
// ortalamasindan cikariyor (self-referencing / AC coupling) - iki farkli ADC
// kanalinin kanal-kanal olcum hatasi birbirine karismasin diye. Bias kanali
// (GPIO4) artik sadece TESHIS amacli okunuyor, RMS hesabina KATILMIYOR.
// Bu karar ARTIK NETLESTI (teammate dogrulayip yazili talimat verdi), asagida
// buna gore uygulanmistir.
//
// ⚠️ VRMS_MULTIPLICATION_VALUE: 150 -> 148.8 (project_conf.h'de guncellendi,
// gercek direnc degerlerinden hesaplanan carpan, ekip arkadasinin dosyasinda
// da bu deger kullaniliyordu).
//
// ⚠️ KALIBRASYON YONTEMI: ESP32-C3'un ADC'si RP2040'inki gibi dogrusal
// varsayilamaz (bkz. CLAUDE.md "gercek/uretim ADC yontemi karari") - bu
// yuzden sabit conversion_factor (3.28/4096) yerine `adc_cali_*` API'siyle
// elde edilen KALIBRE EGIM (V/sayim) kullaniliyor. Performans icin bu egim
// HER ORNEKTE DEGIL, pencere basina BIR KERE hesaplaniyor (ekip arkadasinin
// optimizasyonu) - RMS ham ADC sayilariyla hesaplanip en sonda tek bir
// carpma ile volt'a cevriliyor.
//
// Ham (decimasyonsuz) mu yoksa desimasyonlu mu RMS'in gercek uretim yontemi
// olacagi main.c'nin ADC ornekleme gorev (task) mimarisi kurulurken
// (Asama 3'un son maddesi) netlesecek - decimateSamples() burada, ADCSampleTask
// tarafindan kullanilmak uzere hazir bekliyor.

// ADC donanimini (oneshot birim + ana ve bias kanallari icin kalibrasyon
// semasi) hazirlar. main.c'nin gorev yapisi kurulurken cagrilacak.
uint8_t initADC(void);

// Ana sinyal (ADC_READ_PIN) kanalindan tek bir ham ornek okur -
// ADCSampleTask'in FIFO'yu doldururken kullanacagi dusuk seviye fonksiyon.
// ⚠️ readBiasADCSample() KALDIRILDI: RMS artik self-referencing yontemle
// (bufferin kendi ortalamasindan) hesaplandigi icin bias/GPIO4 kanali hic
// kullanilmiyordu - kullanicinin/hocanin istegiyle tamamen cikarildi,
// gereksiz ADC okuma dongusu kalmadi.
uint16_t readMainADCSample(void);

// initADC() sirasinda bir kere hesaplanan kalibre egim (V/sayim, ana kanal).
// DC/sabit gerilim testlerinde ham sayim ortalamasini voltaja cevirmek icin
// main.c'nin canli ADC test dongusunde kullanilir (calculateVRMS sadece AC/
// dalgali bileseni olcer, DC seviye icin bu gerekir).
float getMainVoltsPerCount(void);

// initADC() sirasinda gercek olculen ornekleme hizina gore hesaplanan
// pencere boyutu (ornek sayisi, VRMS_SAMPLE_SIZE'i asmaz) - ADCSampleTask
// bildirim (notify) esigi ve ADCReadTask'in FIFO'dan cekecegi ornek sayisi
// olarak kullanilir, dev'deki sabit VRMS_SAMPLE_SIZE'in yerini alir.
int getWindowSampleCount(void);

// initADC() sirasinda olculen gercek ornekleme hizi (Hz) - calisirken
// gozlemlenen gercek pencere suresini bu degerden hesaplanan "beklenen"
// sureyle karsilastirip sapma (drift) olup olmadigini gormek icin main.c
// tarafindan kullanilir.
float getMeasuredSampleRateHz(void);

// Ham ornek dizisini `factor` boyutunda bloklar halinde ortalayarak (blok
// ortalama / desimasyon) gurultuyu azaltir. Ekip arkadasinin adc_vrms_test'teki
// decimate() fonksiyonunun birebir portu. Donen deger, out dizisine yazilan
// eleman sayisidir (n / factor).
int decimateSamples(const uint16_t *in, int n, int factor, float *out);

uint16_t calculateVariance(uint16_t *buffer, uint16_t size);
// NOT: bias_voltage parametresi artik CAPRAZ KANAL referansi olarak
// KULLANILMIYOR - RMS, bufferin KENDI ortalamasindan (self-referencing)
// hesaplaniyor. Imza, main.c'nin (henuz portlanmamis) cagri seklini
// degistirmemek icin dev ile ayni birakildi, bias_voltage parametresi
// gecerli bir deger olarak KULLANILMAYIP yoksayiliyor.
float calculateVRMS(uint16_t *buffer, size_t size, float bias_voltage);
// Ekip arkadasinin adc_vrms_test'te GERCEK URETIM YONTEMI olarak isaret
// ettigi hesap - desimasyonlu (blok-ortalamali) self-referencing RMS.
// ADCReadTask'in asil VRMS hesabinda bu cagriliyor, calculateVRMS() (ham)
// hala rapor/karsilastirma icin duruyor.
float calculateVRMSDecimated(uint16_t *buffer, size_t size, int decimation_factor);
float getMean(uint16_t *buffer, size_t size);
void calculateVRMSValuesPerSecond(float *vrms_buffer, uint16_t *sample_buf, size_t buffer_size, size_t sample_size_per_vrms_calc, float bias_voltage);
// Halkadaki bir sonraki yazma konumu (mutlak slot indeksi, 0..TH_RECORD_SLOT_COUNT-1)
// ⚠️ xFlashMutex ALMAZ - cagiran taraf sorumludur.
uint16_t getThresholdWriteIndex(void);
void writeThresholdRecord(const struct ThresholdData *record);
#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED
uint8_t detectSuddenAmplitudeChangeWithDerivative(float *sample_buf, size_t buffer_size);
#endif
void setAmplitudeChangeParameters(struct AmplitudeChangeTimerCallbackParameters *ac_data, float *vrms_values_buffer, uint16_t variance, size_t adc_fifo_size, size_t vrms_values_buffer_size_bytes);

#endif
