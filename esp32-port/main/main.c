/* Vercel git entegrasyonu test amacli zararsiz yorum satiri - islevi etkilemez */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "header/project_globals.h"
#include "header/mutex.h"
#include "header/fifo.h"
#include "header/bcc.h"
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/uart.h"
#include "header/adc.h"
#include "header/ota.h"

// Asama 4 (BLE): ble/include altindaki dosyalar. ⚠️ common.h buraya
// BILEREK include EDILMIYOR - "#define TAG ..." icerdigi icin, bu dosyanin
// zaten tanimli olan "static const char *TAG" degiskenini (asagidaki
// pek cok ESP_LOGI(TAG,...) cagrisinda kullaniliyor) bozardi. Gerekli
// NimBLE host basliklari dogrudan, common.h'siz include ediliyor.
#include "gap.h"
#include "gatt_svc.h"
#include "meter_data.h"
#include "nvs_flash.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
void ble_store_config_init(void);

static const char *TAG = "meter_port";

// ============================================================================
// Bu dosya, `dev` branch'teki blink/main.c'nin GERCEK gorev (task) yapisinin
// ESP32-C3 portudur (Asama 3'un son maddesi). O ana kadar main.c sadece
// modulleri tek tek, senkron/sirali cagirarak test eden bir iskeletti -
// artik gercek, surekli calisan, onceliklendirilmis FreeRTOS gorevlerinden
// olusuyor. Onceki test bloklari (spiflash/uart/adc testleri) KALDIRILDI -
// hepsi zaten fiziksel olarak dogrulanmisti (bkz. CLAUDE.md), onlarin
// yerini artik gercek calisma zamani davranisi aliyor.
//
// dev'den KASITLI FARKLAR (asagida ilgili fonksiyonlarda da tekrar not
// dusulmustur):
// 1) UART ISR+MessageBuffer YERINE, task-context'te byte-byte cerceveleme:
//    ESP-IDF'in uart_driver_install() zaten RX'i donanim kesmesiyle arka
//    planda tamponluyor - dev'in Pico SDK'ya ozel elle yazilmis ISR'ina
//    (uart_receive_interrupt_handler) gerek yok. Ayni mesaj-sinirlama
//    mantigi (LINE_FEED VEYA ETX+BCC ile biten mesaj) burada task icinde,
//    uart_read_bytes() ile birebir ayni state machine olarak calisiyor.
// 2) WatchdogTask YOK - ESP-IDF'in kendi Task Watchdog Timer'i (TWDT,
//    esp_task_wdt.h) zaten ayni "kritik task'lar periyodik olarak kendini
//    isaretlesin, isaretlemeyen olursa sistemi resetle" mantigini native
//    olarak sagliyor. ADCSampleTask/ADCReadTask/UARTTask kendi iclerinde
//    esp_task_wdt_add()/esp_task_wdt_reset() cagiriyor - ayri bir "kontrol
//    eden" task'a gerek yok (bkz. project_conf.h'deki WDT_FLAG_*/
//    task_health_flags - bu eski bitmask mekanizmasi artik KULLANILMIYOR,
//    TWDT'nin kendisi bu isi native yapiyor).
// 3) vTaskCoreAffinitySet cagrilari YOK - ESP32-C3 zaten tek cekirdekli,
//    "hangi gorev hangi core'da" sorusu anlamsizlasiyor.
// 4) vTaskStartScheduler() cagrilmiyor - ESP-IDF, app_main()'i scheduler
//    ZATEN CALISIRKEN cagirir (Pico SDK'nin aksine, orada scheduler'i biz
//    manuel baslatiyorduk). app_main() donebilir, olusturdugumuz gorevler
//    calismaya devam eder.
// 5) StatusLedTask, dev'deki gibi CONF_THRESHOLD_PIN_ENABLED'a gore
//    kosullu DEGIL - bizim kartta threshold pin VE status LED ikisi de
//    ayri, gercek donanim oldugu icin HER ZAMAN calisiyor (bkz. CLAUDE.md
//    "Kesinlesmis kararlar").
// 6) StatusLedTask'in dongu periyodu 2ms yerine 10ms - ESP-IDF'in
//    varsayilan FreeRTOS tick hizi (100Hz) 2ms hassasiyeti veremiyor,
//    pattern dizilerindeki sayilar artik dogrudan milisaniye olarak
//    yorumlaniyor (LED sadece teshis amacli, birebir zamanlama kritik
//    degil).
// ============================================================================

// ---------------------------------------------------------------------------
// UART RX cerceveleme - dev'in ISR+MessageBuffer'ina esdeger (bkz. yukaridaki
// madde 1), TASK baglaminda calisiyor.
// ---------------------------------------------------------------------------
static uint8_t s_rx_buf[RX_BUFFER_SIZE];

// dev'deki uart_receive_interrupt_handler'in rx_index/waiting_for_bcc state
// machine'iyle BIREBIR AYNI mantik: LINE_FEED ile biten kisa istekler VEYA
// ETX+BCC ile biten cerceveli mesajlar tam okununca donuyor. Zaman asiminda
// (timeout_ticks icinde hicbir tam mesaj gelmezse) 0 donuyor.
static size_t uart_receive_message(TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    size_t rx_index = 0;
    bool waiting_for_bcc = false;

    while ((xTaskGetTickCount() - start) < timeout_ticks)
    {
        uint8_t ch;
        int n = uart_read_bytes(UART_PORT_NUM, &ch, 1, pdMS_TO_TICKS(20));
        if (n <= 0)
        {
            continue;
        }

        if (rx_index >= RX_BUFFER_SIZE - 1)
        {
            led_blink_pattern(LED_ERROR_CODE_RX_BUFFER_OVERFLOW_ISR, false);
            return 0;
        }

        s_rx_buf[rx_index++] = ch;

        if (waiting_for_bcc)
        {
            s_rx_buf[rx_index] = '\0';
            return rx_index;
        }

        if (ch == LINE_FEED)
        {
            s_rx_buf[rx_index] = '\0';
            return rx_index;
        }
        else if (ch == ETX_CHAR)
        {
            waiting_for_bcc = true;
        }
    }

    return 0;
}

// ---------------------------------------------------------------------------
// UARTTask - dev'deki vUARTTask ile ayni state machine (kimlik dogrulama ->
// readout/programlama modu -> checkListeningData dispatch). Tek fark,
// mesaj alma cagrisi (xMessageBufferReceive yerine uart_receive_message).
// ---------------------------------------------------------------------------
static void vUARTTask(void *pvParameters)
{
    (void)pvParameters;
    char identify_response_buf[IDENTIFICATION_RESPONSE_BUFFER_SIZE];
    size_t identify_response_len = 0;
    uint8_t message_retry_count = 0;
    uint8_t hex_baud_rate = 0;
    int8_t requested_mode = -1;
    size_t received_bytes;

    esp_task_wdt_add(NULL);

    while (1)
    {
        received_bytes = uart_receive_message(pdMS_TO_TICKS(2000));
        esp_task_wdt_reset();

        if (received_bytes > 0)
        {
            PRINTF("---> %.*s\n", (int)received_bytes, s_rx_buf);

            vTaskDelay(pdMS_TO_TICKS(250));
            if (control_serial_number(s_rx_buf, received_bytes) == true)
            {
                identify_response_len = create_identify_response_message(identify_response_buf, sizeof(identify_response_buf));

                if (identify_response_len >= sizeof(identify_response_buf))
                {
                    PRINTF("Identification response buffer overflow!\n");
                    sendErrorMessage((char *)"IDRESPONSEBUFOVERFLOW");
                    continue;
                }

                while (message_retry_count < MAX_MESSAGE_RETRY_COUNT)
                {
                    uart_write_bytes(UART_PORT_NUM, identify_response_buf, strlen(identify_response_buf));
                    PRINTF("<--- %s", identify_response_buf);
                    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);

                    received_bytes = uart_receive_message(pdMS_TO_TICKS(1500));

                    if (received_bytes > 0)
                    {
                        PRINTF("---> %.*s\n", (int)received_bytes, s_rx_buf);
                        break;
                    }
                    else
                    {
                        PRINTF("No message received after identification within timeout.\n");
                        message_retry_count++;
                    }
                }

                if (message_retry_count >= MAX_MESSAGE_RETRY_COUNT)
                {
                    PRINTF("Max message retry count reached. Aborting identification process.\n");
                    message_retry_count = 0;
                    led_blink_pattern(LED_ERROR_CODE_MESSAGE_TIMEOUT, false);
                    continue;
                }

                vTaskDelay(pdMS_TO_TICKS(250));
                message_retry_count = 0;
                hex_baud_rate = exract_baud_rate_and_mode_from_message(s_rx_buf, received_bytes, &requested_mode);
                set_device_baud_rate(hex_baud_rate);

                if (requested_mode == REQUEST_MODE_LONG_READ || requested_mode == REQUEST_MODE_SHORT_READ)
                {
                    PRINTF("Request is readout\n");
                    send_readout_message(requested_mode);
                    set_init_baud_rate();
                    continue;
                }
                else if (requested_mode == REQUEST_MODE_PROGRAMMING)
                {
                    PRINTF("Request is programming mode\n");
                    send_programming_acknowledgement();
                }
                else
                {
                    PRINTF("Request mode is invalid, ignoring message.\n");
                    set_init_baud_rate();
                    led_blink_pattern(LED_ERROR_CODE_INVALID_REQUEST_MODE, false);
                    continue;
                }

                // IEC62056-21: toplam 30sn sessizlik programlama modunu bitirir.
                TickType_t inner_idle_start = xTaskGetTickCount();
                while (1)
                {
                    received_bytes = uart_receive_message(pdMS_TO_TICKS(3000));
                    esp_task_wdt_reset();

                    if (received_bytes == 0)
                    {
                        if ((xTaskGetTickCount() - inner_idle_start) < pdMS_TO_TICKS(30000))
                        {
                            continue;
                        }
                    }
                    else
                    {
                        inner_idle_start = xTaskGetTickCount();
                    }

                    if (received_bytes == 0 || is_message_break_command(s_rx_buf))
                    {
                        PRINTF("No message received within timeout, ending programming mode.\n");
                        set_init_baud_rate();
                        uart_flush_input(UART_PORT_NUM);
                        break;
                    }

                    PRINTF("---> %.*s\n", (int)received_bytes, s_rx_buf);

                    switch (checkListeningData(s_rx_buf, received_bytes))
                    {
                    case DataError:
                        sendErrorMessage((char *)"DATAERROR");
                        break;

                    case BCCError:
                        sendErrorMessage((char *)"BCCERROR");
                        break;

                    case Password:
                        passwordHandler(s_rx_buf);
                        break;

#if CONF_LOAD_PROFILE_ENABLED
                    case Reading:
                        send_load_profile_records(s_rx_buf);
                        break;
#endif
#if CONF_TIME_SET_ENABLED
                    case TimeSet:
                        setTimeFromUART(s_rx_buf);
                        break;
#endif
#if CONF_DATE_SET_ENABLED
                    case DateSet:
                        setDateFromUART(s_rx_buf);
                        break;
#endif
#if CONF_PRODUCTION_INFO_ENABLED
                    case ProductionInfo:
                        sendProductionInfo();
                        break;
#endif
#if CONF_THRESHOLD_ENABLED
                    case SetThreshold:
                        setThresholdValue(s_rx_buf);
                        break;
#endif
#if CONF_THRESHOLD_PIN_ENABLED
                    case ThresholdPin:
                        resetThresholdPIN();
                        break;
#endif
#if CONF_TIME_READ_ENABLED
                    case ReadTime:
                        readTime();
                        break;
#endif
#if CONF_DATE_READ_ENABLED
                    case ReadDate:
                        readDate();
                        break;
#endif
#if CONF_SERIAL_NUMBER_READ_ENABLED
                    case ReadSerialNumber:
                        readSerialNumber();
                        break;
#endif
#if CONF_VRMS_MAX_READ_ENABLED
                    case ReadLastVRMSMax:
                        sendLastVRMSXValue(ReadLastVRMSMax);
                        break;
#endif
#if CONF_VRMS_MIN_READ_ENABLED
                    case ReadLastVRMSMin:
                        sendLastVRMSXValue(ReadLastVRMSMin);
                        break;
#endif
#if CONF_VRMS_MEAN_READ_ENABLED
                    case ReadLastVRMSMean:
                        sendLastVRMSXValue(ReadLastVRMSMean);
                        break;
#endif
#if CONF_THRESHOLD_OBIS_ENABLED
                    case GetThresholdObis:
                        sendThresholdObis();
                        break;
#endif
                    default:
                        sendErrorMessage((char *)"UNSUPPORTEDOPERATION");
                        break;
                    }
                }
            }
            else
            {
                PRINTF("SN is invalid, ignoring message.\n");
                led_blink_pattern(LED_ERROR_CODE_INVALID_SERIAL_NUMBER, true);
            }
        }
        // 2sn zaman asimi bosta beklerken normal kalp atisi - log yok.
    }
}

// ---------------------------------------------------------------------------
// ADCSampleTask - dev'deki vADCSampleTask ile ayni mantik (FIFO'yu doldur).
//
// ⚠️ BIAS OKUMA TAMAMEN KALDIRILDI (kullanicinin/hocanin istegiyle): RMS
// artik self-referencing yontemle (bufferin kendi ortalamasindan) hesaplandigi
// icin bias/GPIO4 kanali hic kullanilmiyordu - readBiasADCSample() cagrisi,
// bias_buffer/bias_voltage tamamen cikarildi (bkz. adc.c/adc.h/globals.c).
//
// ⚠️ ADCReadTask'A HABER VERME (notify) ARTIK SANIYEDE 1'E SABIT (kullanicinin
// istegiyle): eskiden bias tamponunun dolmasina (dolayisiyla ana pencere
// boyutuna, ~240ms) bagliydi, bu da ADCReadTask'i saniyede ~4 kere tetikliyordu.
// Artik ana FIFO'yu doldurmaya ARALIKSIZ devam ediyoruz (pencere boyutu/50Hz-
// tam-periyot hesabi DEGISMEDI, ADCReadTask hala getWindowSampleCount() kadar
// "son" ornegi cekiyor), ama xTaskNotifyGive() dogrudan 1 saniyelik bir
// zamanlayiciya bagli - VRMS hesabi artik gercekten saniyede 1 tetikleniyor.
//
// ⚠️ KASITLI FARK: dev, RP2040'ta 2000Hz FreeRTOS tick hizinda "tick basina
// 1 ornek" ile calisiyordu (donanim tick'ine kilitli, hassas 2000Hz). ESP-IDF
// varsayilan tick hizi (100Hz) bu hassasiyeti veremiyor - sdkconfig.defaults'ta
// CONFIG_FREERTOS_HZ=1000 yapilarak 1ms tick'e cikarildi.
//
// ⚠️ GERCEK BIR HATA BURADA YAKALANIP DUZELTILDI (dev'de yok, cunku dev
// cift cekirdekliydi ve bu gorev kendi ozel core'undaydi): ilk taslakta bu
// gorev HICBIR YERDE bloklamiyor/vTaskDelay yapmiyordu (surekli/tight-loop
// oneshot okuma). TEK CEKIRDEKTE bu, en yuksek oncelikli (6) bu gorevin
// CPU'yu SONSUZA KADAR tekelinde tutmasi demek - FreeRTOS'ta daha DUSUK
// oncelikli hicbir gorev (UARTTask, StatusLedTask, hatta idle task) BIR
// KERE BILE calisamazdi, cunku bir tick'i asla birakmiyordu. Bu, sadece
// yavaslama degil, sistemin tamamen kilitlenmesi anlamina gelirdi. Duzeltme:
// her BURST_SIZE ornekte bir, 1 tick (CONFIG_FREERTOS_HZ=1000'de 1ms)
// vTaskDelay ile CPU digger gorevlere birakiliyor - bu, ornekleme hizini
// makul (saniyede birkac bin ornek) seviyede tutarken sistemin kilitlenmesini
// engelliyor.
//
// Gercek uretim icin daha ileri adim: ADC continuous/DMA modu (Asama 2
// Madde 8'de arastirildi, 83kHz'e kadar destekliyor) - donanim zamanlamali
// oldugu icin CPU'yu hic mesgul etmeden, bu yield sorununu da kokten
// cozer. Simdilik (Faz 1) bu basit oneshot+burst-yield yaklasimi kullanildi,
// ileride iyilestirilebilir (bkz. CLAUDE.md).
// ---------------------------------------------------------------------------
static void vADCSampleTask(void *pvParameters)
{
    (void)pvParameters;

    // ADCReadTask'a "yeni pencere hazir, VRMS hesapla" haberi artik burada,
    // sabit 1 saniyelik bir zamanlayiciyla veriliyor (bkz. yukaridaki not).
    int64_t notify_last_us = esp_timer_get_time();

    esp_task_wdt_add(NULL);

    while (1)
    {
        for (int i = 0; i < ADC_SAMPLE_BURST_SIZE; i++)
        {
            uint16_t adc_sample = readMainADCSample();

            bool is_added = addToFIFO(&adc_fifo, adc_sample);
            if (!is_added)
            {
                removeFirstElementAddNewElement(&adc_fifo, adc_sample);
            }
        }

        int64_t now_us = esp_timer_get_time();

        if (now_us - notify_last_us >= 1000000)
        {
            xTaskNotifyGive(xADCHandle);
            notify_last_us = now_us;
        }

        esp_task_wdt_reset();
        vTaskDelay(1); // diger gorevlere (UART/LED/RTC/idle) CPU firsati ver
    }
}

// ---------------------------------------------------------------------------
// ADCReadTask - dev'deki vADCReadTask ile ayni mantik (VRMS/varyans hesabi,
// esik kontrolu, load profile periyodunda flash'a yazma).
// ---------------------------------------------------------------------------
static void vADCReadTask(void *pvParameters)
{
    (void)pvParameters;
    static uint16_t adc_samples_buffer[VRMS_SAMPLE_SIZE];
    static float vrms_values_per_second[VRMS_SAMPLE_SIZE / SAMPLE_SIZE_PER_VRMS_CALC];
    static float vrms_buffer[VRMS_BUFFER_SIZE];
    uint16_t vrms_buffer_count = 0;
    // ⚠️ ESP32'ye ozel ekleme (dev'de yok): ADCSampleTask artik dev'deki
    // gibi tam 1Hz'de degil, degisken/daha yuksek bir hizda bildirim
    // yapabiliyor (bkz. yukaridaki ADCSampleTask notu) - ayni dakikada
    // current_time.sec==0 kosulu birden fazla kez dogru olup flash'a
    // TEKRAR TEKRAR yazilmasini (veri bozulmasi degil ama gereksiz/erken
    // tetiklenme) onlemek icin "bu dakika icin zaten yazdik mi" kenar
    // tespiti eklendi.
    int last_load_profile_write_min = -1;

    esp_task_wdt_add(NULL);

    while (1)
    {
        uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));

        if (notif == 0)
        {
            PRINTF("ADC READ TASK: No notification received from ADC SAMPLE TASK within timeout.\r\n");
            continue;
        }

        esp_task_wdt_reset();

        // dev'deki sabit VRMS_SAMPLE_SIZE yerine, gercek olculen hiza gore
        // hesaplanan pencere boyutu kullaniliyor (bkz. ADCSampleTask'taki
        // ayni notu, adc.c/getWindowSampleCount()).
        uint16_t window_samples = (uint16_t)getWindowSampleCount();

        if (xSemaphoreTake(xFIFOMutex, pdMS_TO_TICKS(250)) == pdTRUE)
        {
            getLastNElementsToBuffer(&adc_fifo, adc_samples_buffer, window_samples);
            xSemaphoreGive(xFIFOMutex);
        }
        else
        {
            PRINTF("ADC READ TASK: FIFO MUTEX CANNOT BE TAKEN!\r\n");
            led_blink_pattern(LED_ERROR_CODE_FIFO_MUTEX_NOT_TAKEN, false);
            if (current_time.sec == 0 && current_time.min % load_profile_record_period == 0)
            {
                memset(vrms_buffer, 0, sizeof(vrms_buffer));
                vrms_buffer_count = 0;
                PRINTF("ADC READ TASK: buffer content is deleted\r\n");
            }
            continue;
        }

        // Gercek uretim hesabi: desimasyonlu (16 orneklik blok-ortalamali)
        // self-referencing RMS - ekip arkadasinin adc_vrms_test'te ozellikle
        // isaret ettigi yontem (bkz. adc.h/adc.c). calculateVRMS() (ham,
        // decimasyonsuz) rapor/karsilastirma icin hala mevcut ama artik
        // burada cagrilmiyor.
        float vrms = calculateVRMSDecimated(adc_samples_buffer, window_samples, ADC_DECIMATION_FACTOR);
        // ⚠️ Ayri bir "saniyede 1 bas" freni ARTIK GEREKMIYOR ve KALDIRILDI:
        // bu satir eskiden (bildirim hala ~4x/sn geldigi donemde) kendi basina
        // bir zamanlayiciyla saniyede 1'e dusuruluyordu. Ama ADCSampleTask'in
        // kendisi artik zaten saniyede 1 bildirim gonderiyor (bkz. yukaridaki
        // notify degisikligi) - burada AYRICA bagimsiz bir zamanlayici tutmak,
        // iki farkli 1 saniyelik saatin (biri ADCSampleTask'ta, biri burada)
        // birbirine kilitli OLMAMASI yuzunden ara sira faz kaymasina ve
        // WRITE DEBUG TASK'in duzenli saniye tikleriyle hizali gitmeyip bir
        // basip bir atlamasina sebep oluyordu. Simdi dogrudan basiliyor.
        PRINTF("vrms is: %f\r\n", vrms);

        // BLE (kullanicinin istegiyle): periyodik max/min/mean'in disinda,
        // HER pencerede (artik saniyede 1 - bkz. ADCSampleTask'taki notify
        // degisikligi) taze hesaplanan ham VRMS degerini de anlik olarak
        // disariya veriyoruz.
        vrms_instant = vrms;
        send_vrms_instant_indication();

#if CONF_THRESHOLD_ENABLED || CONF_THRESHOLD_PIN_ENABLED
        uint16_t variance = calculateVariance(adc_samples_buffer, window_samples);
#endif
        // bias_voltage KALDIRILDI (bkz. adc.c) - calculateVRMS() bu parametreyi
        // zaten yoksayiyordu (self-referencing yontem), imza uyumlulugu icin
        // sadece 0.0f geciliyor.
        calculateVRMSValuesPerSecond(vrms_values_per_second, adc_samples_buffer, window_samples, SAMPLE_SIZE_PER_VRMS_CALC, 0.0f);

        vrms_buffer[(vrms_buffer_count++) % VRMS_BUFFER_SIZE] = vrms;

#if CONF_THRESHOLD_PIN_ENABLED || CONF_THRESHOLD_ENABLED
        if (vrms >= (float)getVRMSThresholdValue())
        {
#if CONF_THRESHOLD_PIN_ENABLED
            setThresholdPIN();
#endif
#if CONF_THRESHOLD_ENABLED
            writeThresholdRecord(vrms, variance);
#endif
        }
#endif

        if (current_time.sec == 0 && current_time.min % load_profile_record_period == 0 &&
            last_load_profile_write_min != current_time.min)
        {
            last_load_profile_write_min = current_time.min;

            PRINTF("ADC READ TASK: minute is multiple of %d. write flash block is running...\r\n", load_profile_record_period);

            if (vrms_buffer_count > VRMS_BUFFER_SIZE)
            {
                vrms_buffer_count = VRMS_BUFFER_SIZE;
            }

            VRMS_VALUES_RECORD vrms_values = vrmsSetMinMaxMean(vrms_buffer, vrms_buffer_count);
            PRINTF("ADC READ TASK: calculated VRMS values.\r\n");

            SPIWriteToFlash(&vrms_values);
            PRINTF("ADC READ TASK: writing flash memory process is completed.\r\n");

            // BLE (Asama 4): vrms_max_last/min/mean gercekten burada
            // guncelleniyor (SPIWriteToFlash icinde) - RS485'teki 32.7.0/
            // 52.7.0/72.7.0 ile AYNI kaynak, ayni tazelik/periyot. Abone
            // bir telefon varsa bildirim de burada gonderiliyor.
            update_meter_live_data();
            send_vrms_indication();

            memset(vrms_buffer, 0, sizeof(vrms_buffer));
            vrms_buffer_count = 0;
            PRINTF("ADC READ TASK: buffer content is deleted\r\n");
        }
    }
}

// ---------------------------------------------------------------------------
// WriteDebugTask (dev'deki isim korunuyor, ama icerik sadece RTC okuyup
// current_time'i guncelleyen basit bir gorev - dev'deki vGetRTCTask).
// ---------------------------------------------------------------------------
static void vGetRTCTask(void *pvParameters)
{
    (void)pvParameters;
    TickType_t startTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    while (1)
    {
        vTaskDelayUntil(&startTime, xFrequency);

        if (getTimePt7c4338(&current_time))
        {
            PRINTF("NEW FIRMWARE WRITE DEBUG TASK: The Time is: %02u.%02u.20%02u %02u:%02u:%02u\r\n",
                   current_time.day, current_time.month, current_time.year,
                   current_time.hour, current_time.min, current_time.sec);
        }
        else
        {
            PRINTF("WRITE DEBUG TASK: RTC read error!\r\n");
        }

        // BLE (kullanicinin istegiyle): bos bellek degeri gercekten
        // degistiyse abone bir telefona anlik bildirim gonder - bu gorev
        // zaten 1sn'de bir calistigi icin ek bir gorev/zamanlayiciya
        // gerek kalmadi.
        send_status_indication();
    }
}

// ---------------------------------------------------------------------------
// ResetTask - dev'deki vResetTask ile ayni (RTC senkronu + reset pulse).
// rtc_set_datetime() cagrisi KALDIRILDI (bkz. uart.c'deki ayni not - Pico
// SDK'ya ozel ikinci/dahili RTC'nin ESP32'de karsiligi yok).
// ---------------------------------------------------------------------------
static void vResetTask(void *pvParameters)
{
    (void)pvParameters;
    while (1)
    {
        getTimePt7c4338(&current_time);
        gpio_set_level(RESET_PULSE_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(RESET_PULSE_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(INTERVAL_MS));
    }
}

// ---------------------------------------------------------------------------
// StatusLedTask - dev'deki vStatusLedTask ile ayni desen mantigi, ama:
// - CONF_THRESHOLD_PIN_ENABLED'a gore KOSULLU DEGIL (bkz. dosya basindaki
//   madde 5 - bizim kartta ikisi de ayri, gercek pin).
// - 2ms yerine 10ms dongu periyodu (bkz. madde 6).
// ---------------------------------------------------------------------------
static void vStatusLedTask(void *pvParameters)
{
    (void)pvParameters;
    uint16_t step_index = 0;
    uint32_t elapsed_ms_in_step = 0;
    int last_pattern_id = -1;
    const uint16_t LOOP_PERIOD_MS = 10;
    const TickType_t xFrequency = pdMS_TO_TICKS(LOOP_PERIOD_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (current_pattern_id != last_pattern_id)
        {
            last_pattern_id = current_pattern_id;
            step_index = 0;
            elapsed_ms_in_step = 0;
            gpio_set_level(STATUS_LED_PIN, 1);
        }

        const LedPattern *p = &patterns[current_pattern_id];

        if (elapsed_ms_in_step >= p->sequence[step_index])
        {
            elapsed_ms_in_step = 0;
            step_index++;
            if (step_index >= p->length)
            {
                if (play_once)
                {
                    play_once = false;
                    current_pattern_id = 0;
                }
                step_index = 0;
            }

            gpio_set_level(STATUS_LED_PIN, (step_index % 2) == 0 ? 1 : 0);
        }

        elapsed_ms_in_step += LOOP_PERIOD_MS;
    }
}

// ---------------------------------------------------------------------------
// BLE (Asama 4) - ble_meter_test'teki main.c'nin dogrulanmis kalibiyla ayni,
// sadece "meter_data_task" (sahte veri ureten ayri gorev) YOK - artik gercek
// VRMS guncellemesi ADCReadTask'in kendi icinde, SPIWriteToFlash'tan hemen
// sonra tetikleniyor (bkz. yukaridaki ilgili yorum).
// ---------------------------------------------------------------------------
static void on_ble_stack_reset(int reason)
{
    ESP_LOGI(TAG, "nimble stack reset, reset reason: %d", reason);
}

static void on_ble_stack_sync(void)
{
    adv_init();
}

static void nimble_host_config_init(void)
{
    ble_hs_cfg.reset_cb = on_ble_stack_reset;
    ble_hs_cfg.sync_cb = on_ble_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_store_config_init();
}

static void nimble_host_task(void *param)
{
    (void)param;
    ESP_LOGI(TAG, "nimble host task basladi!");
    nimble_port_run(); // nimble_port_stop() cagrilana kadar donmez
    vTaskDelete(NULL);
}

static void init_ble(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nvs_flash_init basarisiz: %d - BLE devre disi kalacak", ret);
        return;
    }

    meter_data_load_from_nvs();

    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "nimble_port_init basarisiz: %d - BLE devre disi kalacak", ret);
        return;
    }

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    if (gap_init() != 0)
    {
        ESP_LOGE(TAG, "GAP init basarisiz - BLE devre disi kalacak");
        return;
    }
#endif

    if (gatt_svc_init() != 0)
    {
        ESP_LOGE(TAG, "GATT server init basarisiz - BLE devre disi kalacak");
        return;
    }

    nimble_host_config_init();

    if (xTaskCreate(nimble_host_task, "NimBLE Host", 4 * 1024, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "NimBLE host task olusturulamadi");
        return;
    }

    ESP_LOGI(TAG, "BLE baslatildi (cihaz adi: METER-TEST).");
}

// ---------------------------------------------------------------------------
// app_main - dev'deki main() fonksiyonunun ESP-IDF karsiligi: tek seferlik
// donanim/mutex/flash init, sonra gercek gorevleri olusturup donuyor
// (scheduler zaten calisiyor, vTaskStartScheduler() cagrisi YOK).
// ---------------------------------------------------------------------------
void app_main(void)
{
    ESP_LOGI(TAG, "Elektrik sayaci ESP32-C3 portu baslatiliyor (gercek gorev/task mimarisi).");

    // Threshold pin VE status LED ikisi de ayri, gercek pin olarak ilklendiriliyor
    // (dev'deki mutual-exclusion KALDIRILDI - bkz. CLAUDE.md "Kesinlesmis kararlar").
    gpio_reset_pin(THRESHOLD_PIN);
    gpio_set_direction(THRESHOLD_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(THRESHOLD_PIN, 0);

    gpio_reset_pin(STATUS_LED_PIN);
    gpio_set_direction(STATUS_LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(STATUS_LED_PIN, 1);

    gpio_reset_pin(RESET_PULSE_PIN);
    gpio_set_direction(RESET_PULSE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RESET_PULSE_PIN, 0);

    if (!initUART())
    {
        ESP_LOGE(TAG, "UART Init fail! Restarting...");
        esp_restart();
    }

    if (!initADC())
    {
        ESP_LOGE(TAG, "ADC Init fail! Restarting...");
        esp_restart();
    }

    if (!initI2C())
    {
        ESP_LOGE(TAG, "I2C Init fail! Restarting...");
        esp_restart();
    }

    checkSectorContent();
    checkThresholdContent();
    addSerialNumber();
    getFlashContents();
    ESP_LOGI(TAG, "Flash icerigi okundu: sector_data=%d, th_sector_data=%d, vrms_threshold=%d, serial_number='%s'",
             sector_data, th_sector_data, vrms_threshold, serial_number);

    if (!getTimePt7c4338(&current_time))
    {
        ESP_LOGE(TAG, "RTC okunamadi! Restarting...");
        esp_restart();
    }

    if (current_time.dotw < 0 || current_time.dotw > 6)
    {
        current_time.dotw = 2;
    }

    initADCFIFO(&adc_fifo);

    // ⚠️ SIRALAMA ONEMLI, GERCEK BIR COKME BURADA YAKALANDI: setMutexes()
    // setProgramStartDate()'DEN ONCE cagrilmali. dev'in orijinal
    // setProgramStartDate()'i mutex KULLANMIYORDU (boot sirasinda, henuz
    // scheduler/diger task'lar yokken dogrudan flash'a yaziyordu) - ama biz
    // spiflash.c'yi portlarken bu fonksiyona da (proje genelindeki 250ms-
    // timeout mutex desenine uymasi icin, bkz. Asama 3 checklist) xFlashMutex
    // korumasi EKLEDIK. Mutex'ler olusturulmadan (xFlashMutex hala NULL)
    // cagrilirsa, xSemaphoreTake(NULL, ...) "assert failed: xQueueSemaphoreTake
    // (( pxQueue ))" ile cokuyordu - ilk fiziksel testte tam olarak bu oldu.
    if (!setMutexes())
    {
        ESP_LOGE(TAG, "Mutex olusturulamadi! Restarting...");
        esp_restart();
    }

    setProgramStartDate(&current_time);

    // ESP-IDF'in Task Watchdog Timer'i (TWDT) - dev'deki ayri WatchdogTask'in
    // (donanim watchdog + task_health_flags bitmask kontrolu) yerini aliyor,
    // bkz. dosya basindaki madde 2.
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = WATCHDOG_TIMEOUT_MS,
        .idle_core_mask = 0,
        .trigger_panic = true,
    };
    esp_err_t wdt_err = esp_task_wdt_init(&twdt_config);
    if (wdt_err == ESP_ERR_INVALID_STATE)
    {
        // ESP-IDF'in varsayilan sdkconfig'i genelde TWDT'yi app_main()'den
        // ONCE, kendi varsayilan (bizimkinden farkli olabilecek) timeout
        // degeriyle zaten baslatmis oluyor - bu durumda istedigimiz
        // WATCHDOG_TIMEOUT_MS degerini uygulamak icin yeniden yapilandiriyoruz.
        wdt_err = esp_task_wdt_reconfigure(&twdt_config);
    }
    if (wdt_err != ESP_OK)
    {
        ESP_LOGE(TAG, "TWDT baslatilamadi/yapilandirilamadi: %s", esp_err_to_name(wdt_err));
    }

    ESP_LOGI(TAG, "Baslangic tamamlandi. Gorevler olusturuluyor...");

    // ⚠️ SIRALAMA ONEMLI: ADCReadTask, ADCSampleTask'TAN ONCE olusturuluyor.
    // ADCSampleTask (oncelik 6) ADCReadTask'tan (oncelik 5) daha yuksek
    // oncelikli - eger ADCSampleTask once olusturulsaydi, xADCHandle henuz
    // atanmadan (asagidaki ikinci satirdan once) scheduler ADCSampleTask'i
    // hemen calistirabilir ve ilk bildirim denemesinde xTaskNotifyGive(NULL)
    // cagirip cokebilirdi - dev'in orijinal main()'inde de bu yuzden
    // ADCReadTask once olusturuluyordu, ayni siralama korundu.
    xTaskCreate(vADCReadTask, "ADCReadTask", ADC_READ_TASK_STACK_SIZE, NULL, 5, &xADCHandle);
    xTaskCreate(vADCSampleTask, "ADCSampleTask", ADC_SAMPLE_TASK_STACK_SIZE, NULL, 6, &xADCSampleHandle);
    xTaskCreate(vGetRTCTask, "WriteDebugTask", WRITE_DEBUG_TASK_STACK_SIZE, NULL, 5, &xGetRTCHandle);
    xTaskCreate(vUARTTask, "UARTTask", UART_TASK_STACK_SIZE, NULL, 4, &xUARTHandle);
    xTaskCreate(vResetTask, "ResetTask", RESET_TASK_STACK_SIZE, NULL, 7, &xResetHandle);
    xTaskCreate(vStatusLedTask, "StatusLedTask", STATUS_LED_TASK_STACK_SIZE, NULL, 1, &xStatusLedHandle);

    // Tek cekirdek oldugu icin core affinity ayarina gerek yok (bkz. dosya
    // basindaki madde 3).

    // Asama 4: BLE en son baslatiliyor - donanim/gorev mimarisi zaten
    // ayakta, BLE basarisiz olsa bile (init_ble() kendi icinde hata
    // loglayip erken donuyor) geri kalan sistem calismaya devam eder.
    init_ble();

    // BLE uzerinden firmware guncelleme (OTA) - rollback korumasi acik
    // (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y). Buraya kadar hicbir sey
    // cokmeden gelebildiysek, bu firmware "saglikli" - rollback iptal
    // ediliyor. Bu cagrilmazsa, bootloader bir sonraki resette otomatik
    // olarak bir onceki (OTA'dan once calisan) yazilima geri doner.
    ota_mark_valid();

    ESP_LOGI(TAG, "Tum gorevler olusturuldu. app_main donuyor, scheduler (ESP-IDF) zaten calisiyor.");
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)xTask;
    led_blink_pattern(LED_ERROR_CODE_STACK_OVERFLOW, true);
    ESP_LOGE(TAG, "STACK OVERFLOW: %s! Restarting...", pcTaskName);
    esp_restart();
}
