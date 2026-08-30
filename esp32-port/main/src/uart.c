#include "header/uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_partition.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "header/adc.h"
#include "header/bcc.h"
#include "header/mutex.h"
#include "header/print.h"
#include "header/project_globals.h"
#include "header/rtc.h"
#include "header/spiflash.h"
#include "header/threshold_event.h"

// dev branch'teki blink/src/uart.c dosyasindan uyarlanmistir.
//
// NOT: vUARTTask (asil dongu) ve uart_receive_interrupt_handler (kesme/ISR)
// bu dosyada YOK - dev'de de main.c'de. Bu dosya sadece protokolun
// "yardimci fonksiyonlar" katmani: gelen komutu cozumleme, cevap uretme,
// flash/RTC/GPIO'ya erisim. main.c'nin task yapisi ayri bir asamada
// (Asama 3'un son maddesi) kurulacak.
//
// DEV'DEN BILEREK CIKARILAN/DUZELTILEN SEYLER:
// - rtc_set_datetime(&current_time): RP2040'in KENDI donanim RTC
//   cevre biriminin (PT7C4338 harici cipten AYRI, Pico SDK'ya ozel bir
//   ikinci saat) senkronize edilmesiydi. ESP32-C3'te boyle ikinci bir
//   donanim RTC'ye ihtiyacimiz yok - "current_time" global degiskeni
//   zaten tek gercek kaynagimiz. Bu cagrilar kaldirildi.
// - setTimePt7c4338() cagrilari: rtc.c portlanirken imza degisti
//   (I2C_PORT/I2C_ADDRESS parametreleri artik yok, handle zaten
//   initI2C() icinde baglaniyor) - buna gore guncellendi.
// - setThresholdValue(): dev'de flash_range_erase/program'i KENDI
//   icinde tekrar yaziyordu (spiflash.c'deki updateThresholdSector ile
//   neredeyse ayni islemi tekrarliyordu). Biz zaten portladigimiz
//   setVRMSThresholdValue()+updateThresholdSector() fonksiyonlarini
//   YENIDEN KULLANIYORUZ - kod tekrari yok, ayni sonuc.
// - ⚠️ GERCEK BIR HATA DUZELTILDI - send_reset_dates(): dev'de flash
//   okumasini koruyan mutex take/give bloğu YORUM SATIRINA alinmisti
//   ("neden" bilinmiyor) AMA dongunun icinde, hicbir Take'e karsilik
//   gelmeyen BASIBOS bir xSemaphoreGive(xFlashMutex) kalmisti - bu,
//   baska bir task'in gercekten tuttugu mutex'i yanlislikla "birakmis"
//   gibi gorunmesine (tanimsiz davranis) yol acabilirdi. Burada okumanin
//   TAMAMI tek, dogru bir Take/Give bloguna alindi, o basibos Give
//   satiri silindi.

typedef enum
{
    CMD_TYPE_ANY,
    CMD_TYPE_READ,
    CMD_TYPE_WRITE
} CommandType;

typedef struct
{
    const char *obis_code;
    enum ListeningStates state;
    CommandType type;
} ObisCommand;

static const ObisCommand command_table[] = {
    {"P.01", Reading, CMD_TYPE_ANY},
    {"0.9.1", TimeSet, CMD_TYPE_WRITE},
    {"0.9.2", DateSet, CMD_TYPE_WRITE},
    {"0.9.1", ReadTime, CMD_TYPE_READ},
    {"0.9.2", ReadDate, CMD_TYPE_READ},
    {"0.0.0", ReadSerialNumber, CMD_TYPE_READ},
    {"96.1.3", ProductionInfo, CMD_TYPE_ANY},
    {"96.3.12", SetThreshold, CMD_TYPE_WRITE},
    {"96.3.12", GetThresholdObis, CMD_TYPE_READ},
    {"96.77.4*", GetThreshold, CMD_TYPE_ANY},
    {"96.3.10", ThresholdPin, CMD_TYPE_ANY},
    {"9.9.0", GetSuddenAmplitudeChange, CMD_TYPE_ANY},
    {"32.7.0", ReadLastVRMSMax, CMD_TYPE_ANY},
    {"52.7.0", ReadLastVRMSMin, CMD_TYPE_ANY},
    {"72.7.0", ReadLastVRMSMean, CMD_TYPE_ANY},
    {"0.1.2*", ReadResetDates, CMD_TYPE_ANY}};

static const uint8_t password_cmd[4] = {0x01, 0x50, 0x31, 0x02};
static const uint8_t reading_control_cmd[4] = {0x01, 0x52, 0x32, 0x02};
static const uint8_t reading_control_alt_cmd[4] = {0x01, 0x52, 0x35, 0x02};
static const uint8_t writing_control_cmd[4] = {0x01, 0x57, 0x32, 0x02};
static const uint8_t break_command[5] = {0x01, 0x42, 0x30, 0x03, 0x71};

// ---------------------------------------------------------------------------
// Kucuk yardimcilar: dev'in Pico SDK uart_putc()/uart_puts() cagrilarinin
// ESP-IDF karsiligi. Butun dosyada bunlar kullanilacak.
// ---------------------------------------------------------------------------
static inline void uart_putc_esp(uint8_t c)
{
    uart_write_bytes(UART_PORT_NUM, (const char *)&c, 1);
}

static inline void uart_puts_esp(const char *s)
{
    uart_write_bytes(UART_PORT_NUM, s, strlen(s));
}

// UART Initialization - protokolun gercek baglanti ayarlariyla (300 baud,
// 7 veri biti, cift parite, 1 stop biti - defines.h'deki degerler)
uint8_t initUART()
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = DATA_BITS,
        .parity = PARITY,
        .stop_bits = STOP_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(UART_PORT_NUM, RX_BUFFER_SIZE * 2, 0, 0, NULL, 0) != ESP_OK)
    {
        PRINTF("UART INIT ERROR (driver install)!\n");
        return 0;
    }
    if (uart_param_config(UART_PORT_NUM, &uart_config) != ESP_OK)
    {
        PRINTF("UART INIT ERROR (param config)!\n");
        return 0;
    }
    if (uart_set_pin(UART_PORT_NUM, UART0_TX_PIN, UART0_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK)
    {
        PRINTF("UART INIT ERROR (set pin)!\n");
        return 0;
    }

    return 1;
}

uint8_t is_message_break_command(uint8_t *buf)
{
    return (memcmp(buf, break_command, sizeof(break_command)) == 0);
}

// This function check the data which comes when State is Listening, and compares the message to defined strings, and returns a ListeningState value to process the request
enum ListeningStates checkListeningData(uint8_t *data_buffer, uint8_t size)
{
    if (!bccControl(data_buffer, size))
    {
        return BCCError;
    }

    if (memcmp(data_buffer, password_cmd, sizeof(password_cmd)) == 0)
    {
        return Password;
    }

    if (memcmp(data_buffer, break_command, sizeof(break_command)) == 0)
    {
        PRINTF("CHECKLISTENINGDATA: Break command received.\n");
        return BreakMessage;
    }

    const bool is_any_reading_msg = (memcmp(data_buffer, reading_control_cmd, sizeof(reading_control_cmd)) == 0) ||
                                     (memcmp(data_buffer, reading_control_alt_cmd, sizeof(reading_control_alt_cmd)) == 0);
    const bool is_writing_msg = (memcmp(data_buffer, writing_control_cmd, sizeof(writing_control_cmd)) == 0);

    if (is_any_reading_msg || is_writing_msg)
    {
        const char *buffer_as_char = (const char *)data_buffer;
        for (size_t i = 0; i < (sizeof(command_table) / sizeof(command_table[0])); ++i)
        {
            if (strstr(buffer_as_char, command_table[i].obis_code) != NULL)
            {
                bool type_match = false;
                if (command_table[i].type == CMD_TYPE_ANY)
                {
                    type_match = true;
                }
                else if (command_table[i].type == CMD_TYPE_READ && is_any_reading_msg)
                {
                    type_match = true;
                }
                else if (command_table[i].type == CMD_TYPE_WRITE && is_writing_msg)
                {
                    type_match = true;
                }

                if (type_match)
                {
                    PRINTF("CHECKLISTENINGDATA: State %d is accepted.\n", command_table[i].state);
                    return command_table[i].state;
                }
            }
        }
    }

    return DataError;
}

// This function deletes a character from a given string
void deleteChar(uint8_t *str, uint8_t len, char chr)
{
    uint8_t i, j;
    for (i = j = 0; i < len; i++)
    {
        if (str[i] != chr)
        {
            str[j++] = str[i];
        }
    }
    str[j] = '\0';
}

void parseACRequestDate(uint8_t *buffer, uint8_t *start_date, uint8_t *end_date)
{
    char *date_start_ptr = strchr((char *)buffer, '(');
    char *date_end_ptr = strchr((char *)buffer, ')');
    char *date_division_ptr = strchr((char *)buffer, ';');

    if (date_start_ptr == NULL || date_end_ptr == NULL || date_division_ptr == NULL)
    {
        PRINTF("PARSEACREQUESTDATE: Date format is invalid!\n");
        return;
    }

    uint8_t sd_temp[20] = {0};
    uint8_t ed_temp[20] = {0};

    memcpy(sd_temp, date_start_ptr + 1, date_division_ptr - date_start_ptr - 1);
    memcpy(ed_temp, date_division_ptr + 1, date_end_ptr - date_division_ptr - 1);

    deleteChar(sd_temp, strlen((char *)sd_temp), '-');
    deleteChar(sd_temp, strlen((char *)sd_temp), ',');
    deleteChar(sd_temp, strlen((char *)sd_temp), ':');

    deleteChar(ed_temp, strlen((char *)ed_temp), '-');
    deleteChar(ed_temp, strlen((char *)ed_temp), ',');
    deleteChar(ed_temp, strlen((char *)ed_temp), ':');

    memcpy(start_date, sd_temp, 12);
    memcpy(end_date, ed_temp, 12);
}

// This function sets the device's baud rate according to given number like 0,1,2,3,4,5,6
void set_device_baud_rate(uint8_t b_rate_hex)
{
    uint32_t set_baud_rate = 0;

    switch (b_rate_hex)
    {
    case 0x30:
        set_baud_rate = 300;
        break;
    case 0x31:
        set_baud_rate = 600;
        break;
    case 0x32:
        set_baud_rate = 1200;
        break;
    case 0x33:
        set_baud_rate = 2400;
        break;
    case 0x34:
        set_baud_rate = 4800;
        break;
    case 0x35:
        set_baud_rate = 9600;
        break;
    case 0x36:
        set_baud_rate = 19200;
        break;
    default:
        set_baud_rate = 300;
        break;
    }
    uart_set_baudrate(UART_PORT_NUM, set_baud_rate);
}

// ⚠️ get_serial_number_ptr() KALDIRILDI: eskiden flash'tan okunmus RAM
// kopyasinin (serial_number global'i) adresini donduruyordu. Seri no artik
// dogrudan DEVICE_SERIAL_NUMBER makrosundan geliyor (derleme zamani sabiti),
// dolayisiyla ne global'e ne de bir isaretciye gerek var - bkz. spiflash.c'de
// addSerialNumber()'in kaldirildigi yerdeki not.

void set_init_baud_rate()
{
    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
    uart_set_baudrate(UART_PORT_NUM, 300);
    // Clear the RX FIFO to remove any garbage characters received during baud rate switch
    uart_flush_input(UART_PORT_NUM);
}

bool control_serial_number(uint8_t *identification_req_buf, size_t req_size)
{
    uint8_t i;

    // NULL kontrolu artik EN BASTA: eskiden bu kontrol, isaretciyi zaten
    // kullanan memcpy'lardan SONRA geliyordu (yani NULL gelseydi kontrole
    // varmadan cokerdi). Seri no isaretcisi ortadan kalktigi icin geriye
    // sadece gelen mesaj tamponunun kontrolu kaldi.
    if (identification_req_buf == NULL)
    {
        PRINTF("Identification request buffer is NULL!\n");
        return false;
    }

    if (strncmp((char *)identification_req_buf, (char *)"/?!\r\n", req_size) == 0)
    {
        return true;
    }

    for (i = 0; *identification_req_buf != '?'; identification_req_buf++, i++)
    {
        if (i == req_size)
        {
            PRINTF("Identification is wrong!\n");
            return false;
        }
    }

    identification_req_buf++;

    // Kabul edilen iki bicim, derleme zamaninda string birlestirmesiyle
    // olusuyor (METER_FLAG_CODE zaten "ALP") - eskiden bu iki dizge her
    // cagride stack'te memcpy ile kuruluyordu, artik hicbir tampon yok:
    //   "612400080"    -> bayraksiz
    //   "ALP612400080" -> bayrak onekli
    if (strncmp((char *)identification_req_buf, DEVICE_SERIAL_NUMBER, SERIAL_NUMBER_SIZE) == 0 ||
        strncmp((char *)identification_req_buf, METER_FLAG_CODE DEVICE_SERIAL_NUMBER, SERIAL_NUMBER_FLAG_SIZE + SERIAL_NUMBER_SIZE) == 0)
    {
        PRINTF("Serial number is correct.\n");
        return true;
    }

    return false;
}

size_t create_identify_response_message(char *response_buf, size_t buf_size)
{
    memset(response_buf, 0, buf_size);
    return snprintf(response_buf, buf_size, "/%s%d<%d>---(MA-V3)\r\n", METER_FLAG_CODE, METER_MAX_SUPPORTED_BAUDRATE, METER_VERSION);
}

uint8_t exract_baud_rate_and_mode_from_message(uint8_t *msg_buf, size_t msg_len, int8_t *requested_mode)
{
    if (msg_buf == NULL || msg_len < 3)
    {
        PRINTF("Message buffer is NULL or message length is invalid!\n");
        return 0;
    }

    uint8_t baud_rate = msg_buf[2];
    *requested_mode = (int8_t)msg_buf[3];

    return baud_rate;
}

// ---------------------------------------------------------------------------
// Uzun okuma icin ek kayitlar (esik asim + reset kayitlari)
// ---------------------------------------------------------------------------

// Esik asim kayitlari: halkadaki SON 10 kayit, eskiden yeniye dogru
// (*1 = en eski, *10 = en yeni) gonderilir - reset kayitlari (0.1.2*N,
// send_reset_dates) ile AYNI kural.
//
// Kayit alani bir HALKA tampon oldugu icin artik tek bir sektorun icinde
// aranmiyor; YAZMA KONUMUNDAN geriye dogru yurunuyor. Boylece sektor sinirini
// gecen pencereler de dogru toplanir - onceki surumdeki "sadece aktif sektore
// bakilir" kisiti kalkti.
//
// Kayitlarin anlami da degisti: artik saniyelik ornek degil OLAY yaziliyor.
// vrms alani SANTIVOLT (573 = 5.73 V), ikinci alan varyans degil olayin
// DAKIKA cinsinden suresi (65535 = olay hala suruyor).
void send_threshold_records(uint8_t *xor_result)
{
    uint8_t threshold_records_raw[FLASH_RECORD_SIZE * THRESHOLD_RECORD_OBIS_COUNT];
    uint8_t buffer[48] = {0};
    char year[3] = {0};
    char month[3] = {0};
    char day[3] = {0};
    char hour[3] = {0};
    char min[3] = {0};
    char sec[3] = {0};
    uint16_t vrms = 0;
    uint16_t duration = 0;
    int result;

    memset(threshold_records_raw, 0xFF, sizeof(threshold_records_raw));

    const esp_partition_t *threshold_rec_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_THRESHOLD_REC);
    if (threshold_rec_part == NULL)
    {
        PRINTF("SEND THRESHOLD RECORDS: threshold_rec partition bulunamadi!\n");
        sendErrorMessage((char *)"THPARTNOTFOUND");
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        // ⚠️ GERCEK BIR HATA BURADAYDI, BULUNUP DUZELTILDI: burasi her zaman
        // partition'in 0. sektorunden okuyordu, ama yazma tarafi (adc.c/
        // writeThresholdRecordsSectorToFlash) su anki AKTIF sektore
        // (th_sector_data, 16 sektorluk dairesel alanda doner) yaziyor -
        // okuma ve yazma farkli fiziksel sektorlere baktigi icin RS485
        // uzerinden hep bos/eski veri gorunuyordu. Duzeltme: okuma da
        // th_sector_data'yi kullanmaya basladi.
        //
        // getThresholdWriteIndex() kendi icinde mutex ALMAZ; burada zaten
        // tutuyoruz (rekursif olmayan mutex'te ikinci alma kilitlenirdi).
        uint16_t write_index = getThresholdWriteIndex();

        // Halkada geriye dogru: back = THRESHOLD_RECORD_OBIS_COUNT en eski,
        // back = 1 en yeni. i = 0 -> en eski olacak sekilde dolduruyoruz ki
        // asagidaki artan indeks (*1..*10) kronolojik sirayla gitsin.
        for (size_t i = 0; i < THRESHOLD_RECORD_OBIS_COUNT; i++)
        {
            uint16_t back = (uint16_t)(THRESHOLD_RECORD_OBIS_COUNT - i);
            uint16_t slot = thSlotBack(write_index, back, TH_RECORD_SLOT_COUNT);

            esp_partition_read(threshold_rec_part, (size_t)slot * FLASH_RECORD_SIZE,
                               &threshold_records_raw[i * FLASH_RECORD_SIZE], FLASH_RECORD_SIZE);
        }

        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("SEND THRESHOLD RECORDS: Could not take flash mutex!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        sendErrorMessage((char *)"FLASHMUTEXERR");
        return;
    }

    // Indeks 1'den THRESHOLD_RECORD_OBIS_COUNT'a ARTARAK ilerler ve bellekteki
    // sira zaten kronolojik oldugu icin *1 en eski, *10 en yeni kayittir.
    for (size_t i = 0, idx = 1; i < THRESHOLD_RECORD_OBIS_COUNT; i++, idx++)
    {
        size_t offset = i * FLASH_RECORD_SIZE;

        if (threshold_records_raw[offset] == 0xFF || threshold_records_raw[offset] == 0x00)
        {
            result = snprintf((char *)buffer, sizeof(buffer), "96.77.4*%d(00-00-00,00:00:00)(000.00,00000)\r\n", (int)idx);
        }
        else
        {
            snprintf(year, sizeof(year), "%c%c", threshold_records_raw[offset], threshold_records_raw[offset + 1]);
            snprintf(month, sizeof(month), "%c%c", threshold_records_raw[offset + 2], threshold_records_raw[offset + 3]);
            snprintf(day, sizeof(day), "%c%c", threshold_records_raw[offset + 4], threshold_records_raw[offset + 5]);
            snprintf(hour, sizeof(hour), "%c%c", threshold_records_raw[offset + 6], threshold_records_raw[offset + 7]);
            snprintf(min, sizeof(min), "%c%c", threshold_records_raw[offset + 8], threshold_records_raw[offset + 9]);
            snprintf(sec, sizeof(sec), "%c%c", threshold_records_raw[offset + 10], threshold_records_raw[offset + 11]);
            // vrms artik SANTIVOLT (573 = 5.73 V), ikinci alan da varyans degil
            // olayin DAKIKA cinsinden suresi (65535 = olay hala suruyor).
            vrms = threshold_records_raw[offset + 13];
            vrms = (vrms << 8);
            vrms += threshold_records_raw[offset + 12];
            duration = threshold_records_raw[offset + 15];
            duration = (duration << 8);
            duration += threshold_records_raw[offset + 14];

            result = snprintf((char *)buffer, sizeof(buffer), "96.77.4*%d(%s-%s-%s,%s:%s:%s)(%03d.%02d,%05d)\r\n", (int)idx, year, month, day, hour, min, sec, vrms / 100, vrms % 100, duration);
        }

        bccGenerate(buffer, result, xor_result);

        if (result >= (int)sizeof(buffer))
        {
            PRINTF("GETTHRESHOLDRECORD: Buffer Overflow! Sending NACK.\n");
            sendErrorMessage((char *)"THBUFFEROVERFLOW");
        }
        else
        {
            uart_puts_esp((char *)buffer);
        }

        vTaskDelay(pdMS_TO_TICKS(15));
    }

    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
}

// ⚠️ GERCEK BIR STACK TASMASI BURADA YAKALANIP DUZELTILDI: reset_dates_flash
// (4096 byte) yerel/stack degiskeni olarak tanimliydi, ama bu fonksiyonu
// cagiran UARTTask'in stack'i sadece 2048 byte (UART_TASK_STACK_SIZE) -
// yani tek basina bu dizi, tum stack'in 2 katini tasiriyordu. Fiziksel
// testte (gercek gorev mimarisiyle, uzun okuma/-rm komutuyla) bu tam
// olarak gozlemlendi: reset kayitlari bolumune gelindiginde RS485 hattina
// anlamsiz/bozuk baytlar gitmeye basladi, BCC dogrulamasi basarisiz oldu.
// Duzeltme: setProgramStartDate'te (spiflash.c) daha once uygulanan ayni
// desen - buyuk dizi `static` yapilarak stack'ten TAMAMEN cikariliyor
// (BSS'e taşınıyor), boylece UARTTask'in kucuk stack'ini tuketmiyor.
void send_reset_dates(uint8_t *xor_result)
{
    static uint8_t reset_dates_flash[FLASH_SECTOR_SIZE];
    uint8_t reset_dates_raw[RESET_DATES_OBIS_COUNT * FLASH_RECORD_SIZE];
    char date_buffer[32];
    int result;

    memset(reset_dates_raw, 0, sizeof(reset_dates_raw));
    memset(date_buffer, 0, sizeof(date_buffer));

    const esp_partition_t *reset_dates_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, PARTITION_LABEL_RESET_DATES);
    if (reset_dates_part == NULL)
    {
        PRINTF("SEND RESET DATES: reset_dates partition bulunamadi!\n");
        sendErrorMessage((char *)"RDPARTNOTFOUND");
        return;
    }

    // dev'de bu okuma mutex'siz yapiliyordu (take/give yorum satirindaydi) -
    // duzeltildi: okumanin tamami (tarama + kopyalama) tek bir mutex
    // blogu icinde yapiliyor.
    uint16_t idx = 0;
    uint16_t obis_idx = 0;
    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        esp_partition_read(reset_dates_part, 0, reset_dates_flash, FLASH_SECTOR_SIZE);

        while (idx < FLASH_SECTOR_SIZE)
        {
            if (reset_dates_flash[idx] == 0x00 || reset_dates_flash[idx] == 0xFF)
            {
                break;
            }
            idx += FLASH_RECORD_SIZE;
            obis_idx++;
        }

        uint32_t start_offset;
        uint32_t end_offset = idx;

        if (end_offset > FLASH_RECORD_SIZE * RESET_DATES_OBIS_COUNT)
        {
            start_offset = end_offset - (FLASH_RECORD_SIZE * RESET_DATES_OBIS_COUNT);
        }
        else
        {
            start_offset = 0;
        }

        memcpy(reset_dates_raw, reset_dates_flash + start_offset, end_offset - start_offset);

        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("SEND RESET DATES: Could not take flash mutex!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        sendErrorMessage((char *)"FLASHMUTEXERR");
        return;
    }

    for (uint16_t i = 0, obis = 1; i < sizeof(reset_dates_raw); i += FLASH_RECORD_SIZE, obis++)
    {
        if (reset_dates_raw[i] == 0xFF || reset_dates_raw[i] == 0x00)
        {
            result = snprintf(date_buffer, sizeof(date_buffer), "0.1.2*%d(00-00-00,00:00:00)\r\n", obis);
        }
        else
        {
            char year[3] = {(char)reset_dates_raw[i], (char)reset_dates_raw[i + 1], 0x00};
            char month[3] = {(char)reset_dates_raw[i + 2], (char)reset_dates_raw[i + 3], 0x00};
            char day[3] = {(char)reset_dates_raw[i + 4], (char)reset_dates_raw[i + 5], 0x00};
            char hour[3] = {(char)reset_dates_raw[i + 6], (char)reset_dates_raw[i + 7], 0x00};
            char min[3] = {(char)reset_dates_raw[i + 8], (char)reset_dates_raw[i + 9], 0x00};
            char sec[3] = {(char)reset_dates_raw[i + 10], (char)reset_dates_raw[i + 11], 0x00};

            result = snprintf(date_buffer, sizeof(date_buffer), "0.1.2*%d(%s-%s-%s,%s:%s:%s)\r\n", obis, year, month, day, hour, min, sec);

            if (result >= (int)sizeof(date_buffer))
            {
                sendErrorMessage((char *)"DATEBUFFERSMALL");
                continue;
            }
        }

        bccGenerate((uint8_t *)date_buffer, result, xor_result);
        uart_puts_esp(date_buffer);
    }
}

void send_readout_message(uint8_t request_mode)
{
    char readout_line_buffer[32];
    int result = 0;
    uint8_t readout_xor = 0x00;

    memset(readout_line_buffer, 0, sizeof(readout_line_buffer));

    uart_putc_esp(STX);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.0.0(%s)\r\n", DEVICE_SERIAL_NUMBER);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.2.0(%s)\r\n", SOFTWARE_VERSION);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.8.4(%d*min)\r\n", load_profile_record_period);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.9.1(%02d:%02d:%02d)\r\n", current_time.hour, current_time.min, current_time.sec);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "0.9.2(%02d-%02d-%02d)\r\n", current_time.year, current_time.month, current_time.day);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "96.1.3(%s)\r\n", PRODUCTION_DATE);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "96.3.12(%03d)\r\n", getVRMSThresholdValue());
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    if (request_mode == REQUEST_MODE_LONG_READ)
    {
        send_threshold_records(&readout_xor);
        send_reset_dates(&readout_xor);
    }

    if (xSemaphoreTake(xVRMSLastValuesMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "32.7.0(%.2f)\r\n", vrms_max_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts_esp(readout_line_buffer);

        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "52.7.0(%.2f)\r\n", vrms_min_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts_esp(readout_line_buffer);

        result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "72.7.0(%.2f)\r\n", vrms_mean_last);
        bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
        uart_puts_esp(readout_line_buffer);

        xSemaphoreGive(xVRMSLastValuesMutex);
    }
    else
    {
        PRINTF("SEND READOUT MESSAGE: Could not take VRMS last values mutex!\n");
        led_blink_pattern(LED_ERROR_CODE_VRMS_VALUES_MUTEX_NOT_TAKEN, false);
    }

    result = snprintf(readout_line_buffer, sizeof(readout_line_buffer), "!\r\n%c", ETX);
    bccGenerate((uint8_t *)readout_line_buffer, result, &readout_xor);
    uart_puts_esp(readout_line_buffer);

    PRINTF("SETTINGSTATEHANDLER: readout XOR is: %02X.\n", readout_xor);
    uart_putc_esp(readout_xor);
    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
}

void send_programming_acknowledgement()
{
    uint8_t ack_buff[32];
    uint8_t ack_bcc = SOH;

    memset(ack_buff, 0, sizeof(ack_buff));

    int result = snprintf((char *)ack_buff, sizeof(ack_buff), "%cP0%c(%s)%c", SOH, STX, DEVICE_SERIAL_NUMBER, ETX);
    if (result >= (int)sizeof(ack_buff))
    {
        sendErrorMessage((char *)"ACKBUFOVERFLOW");
        return;
    }

    bccGenerate(ack_buff, result, &ack_bcc);

    uart_puts_esp((char *)ack_buff);
    uart_putc_esp(ack_bcc);

    uart_wait_tx_done(UART_PORT_NUM, portMAX_DELAY);
}

uint8_t verifyHourMinSec(uint8_t hour, uint8_t min, uint8_t sec)
{
    if (hour > 23 || min > 59 || sec > 59)
    {
        return 0;
    }
    return 1;
}

uint8_t verifyYearMonthDay(uint8_t year, uint8_t month, uint8_t day)
{
    if (year > 99 || month > 12 || day > 31)
    {
        return 0;
    }
    return 1;
}

// This function sets time via UART
void setTimeFromUART(uint8_t *buffer)
{
    if (!password_correct_flag)
    {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    uint8_t time_buffer[9] = {0};
    uint8_t hour, min, sec;

    char *start_ptr = strchr((char *)buffer, '(');
    if (start_ptr == NULL)
    {
        return;
    }
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');
    if (end_ptr == NULL)
    {
        return;
    }

    strncpy((char *)time_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(time_buffer, strlen((char *)time_buffer), ':');

    hour = (time_buffer[0] - '0') * 10 + (time_buffer[1] - '0');
    min = (time_buffer[2] - '0') * 10 + (time_buffer[3] - '0');
    sec = (time_buffer[4] - '0') * 10 + (time_buffer[5] - '0');

    PRINTF("SETTIMEFROMUART: hour: %d, min: %d, sec: %d\n", hour, min, sec);

    if (verifyHourMinSec(hour, min, sec))
    {
        if (!setTimePt7c4338(sec, min, hour, (uint8_t)current_time.dotw, (uint8_t)current_time.day, (uint8_t)current_time.month, (uint8_t)current_time.year))
        {
            sendErrorMessage((char *)"PT7CTIMENOTSET");
            return;
        }

        if (!getTimePt7c4338(&current_time))
        {
            sendErrorMessage((char *)"PT7CTIMENOTGET");
            return;
        }

        if (current_time.dotw < 0 || current_time.dotw > 6)
        {
            PRINTF("SETTIMEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }

        PRINTF("SETTIMEFROMUART: time was set to: %d:%d:%d\n", current_time.hour, current_time.min, current_time.sec);
        uart_putc_esp(ACK);
    }
    else
    {
        PRINTF("SETTIMEFROMUART: invalid time values!\n");
        sendErrorMessage((char *)"INVALIDTIMEVAL");
    }

    password_correct_flag = false;
}

// This function sets date via UART
void setDateFromUART(uint8_t *buffer)
{
    if (!password_correct_flag)
    {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    uint8_t date_buffer[9] = {0};
    uint8_t year, month, day;

    char *start_ptr = strchr((char *)buffer, '(');
    if (start_ptr == NULL)
    {
        return;
    }
    start_ptr++;
    char *end_ptr = strchr((char *)buffer, ')');
    if (end_ptr == NULL)
    {
        return;
    }

    strncpy((char *)date_buffer, start_ptr, end_ptr - start_ptr);
    deleteChar(date_buffer, strlen((char *)date_buffer), '-');

    year = (date_buffer[0] - '0') * 10 + (date_buffer[1] - '0');
    month = (date_buffer[2] - '0') * 10 + (date_buffer[3] - '0');
    day = (date_buffer[4] - '0') * 10 + (date_buffer[5] - '0');

    PRINTF("SETDATEFROMUART: year: %d, month: %d, day: %d\n", year, month, day);

    if (verifyYearMonthDay(year, month, day))
    {
        if (!setTimePt7c4338((uint8_t)current_time.sec, (uint8_t)current_time.min, (uint8_t)current_time.hour, (uint8_t)current_time.dotw, day, month, year))
        {
            sendErrorMessage((char *)"PT7CDATENOTSET");
            return;
        }
        if (!getTimePt7c4338(&current_time))
        {
            sendErrorMessage((char *)"PT7CDATENOTGET");
            return;
        }

        if (current_time.dotw < 0 || current_time.dotw > 6)
        {
            PRINTF("SETDATEFROMUART: invalid day of the week: %d!\n", current_time.dotw);
            current_time.dotw = 2;
        }

        PRINTF("SETDATEFROMUART: date was set to: %d-%d-%d\n", current_time.year, current_time.month, current_time.day);
        uart_putc_esp(ACK);
    }
    else
    {
        PRINTF("SETDATEFROMUART: invalid date values!\n");
        sendErrorMessage((char *)"INVALIDDATEVAL");
    }

    password_correct_flag = false;
}

// This function generates a production info message and sends it to UART
void sendProductionInfo()
{
    char production_obis_buffer[24] = {0};
    uint8_t production_bcc = STX;

    int result = snprintf(production_obis_buffer, sizeof(production_obis_buffer), "%c96.1.3(%s)\r\n%c", STX, PRODUCTION_DATE, ETX);
    if (result >= (int)sizeof(production_obis_buffer))
    {
        PRINTF("SENDPRODUCTIONINFO: production buffer is too small.\n");
        sendErrorMessage((char *)"SMALLBUFFERSIZE");
        return;
    }

    bccGenerate((uint8_t *)production_obis_buffer, result, &production_bcc);

    uart_puts_esp(production_obis_buffer);
    uart_putc_esp(production_bcc);
}

// This function gets a password and controls the password, if password is true, device sends an ACK message, if not, device sends NACK message
void passwordHandler(uint8_t *buffer)
{
    char *ptr = strchr((char *)buffer, '(');
    if (ptr == NULL)
    {
        sendErrorMessage((char *)"PWFORMATERROR");
        return;
    }
    ptr++;

    if (strncmp(ptr, DEVICE_PASSWORD, 8) == 0)
    {
        uart_putc_esp(ACK);
        password_correct_flag = true;
    }
    else
    {
        sendErrorMessage((char *)"PWNOTCORRECT");
    }
}

void setThresholdValue(uint8_t *data)
{
    if (!password_correct_flag)
    {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }
    PRINTF("SETTHRESHOLDVALUE: threshold value before change is: %d\n", getVRMSThresholdValue());

    char *start_ptr = strchr((char *)data, '(');
    char *end_ptr = strchr((char *)data, ')');

    if (start_ptr == NULL || end_ptr == NULL || end_ptr <= start_ptr)
    {
        PRINTF("SETTHRESHOLDVALUE: Invalid data format\n");
        sendErrorMessage((char *)"DATAFORMATERROR");
        return;
    }

    start_ptr++;
    size_t len = end_ptr - start_ptr;

    if (len == 0 || len >= 4)
    {
        PRINTF("SETTHRESHOLDVALUE: Invalid value length\n");
        sendErrorMessage((char *)"VALUELENGTHERROR");
        return;
    }

    char threshold_val_str[4];
    strncpy(threshold_val_str, start_ptr, len);
    threshold_val_str[len] = '\0';
    uint16_t threshold_val = atoi(threshold_val_str);

    PRINTF("SETTHRESHOLDVALUE: threshold value as string is: %s\n", threshold_val_str);
    PRINTF("SETTHRESHOLDVALUE: threshold value as int is: %d\n", threshold_val);

    if (getVRMSThresholdValue() == threshold_val)
    {
        uart_putc_esp(ACK);
        return;
    }

    // dev'de burada flash_range_erase/program tekrar el ile yapiliyordu -
    // zaten portladigimiz setVRMSThresholdValue()+updateThresholdSector()
    // fonksiyonlarini (mutex.h/spiflash.h) yeniden kullaniyoruz, kod
    // tekrari yok.
    setVRMSThresholdValue(threshold_val);
    updateThresholdSector(th_sector_data);

    uart_putc_esp(ACK);
    PRINTF("SETTHRESHOLDVALUE: ACK send from set threshold value.\n");

    password_correct_flag = false;
}

#if CONF_THRESHOLD_PIN_ENABLED
void resetThresholdPIN()
{
    if (!password_correct_flag)
    {
        sendErrorMessage((char *)"NOPWENTERED");
        return;
    }

    if (getThresholdSetBeforeFlag())
    {
        PRINTF("RESETTHRESHOLDPIN: Threshold PIN set before, resetting pin...\n");

        gpio_set_level(THRESHOLD_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        setThresholdSetBeforeFlag(0);

        PRINTF("RESETTHRESHOLDPIN: ACK send from set threshold pin.\n");
        uart_putc_esp(ACK);
    }
    else
    {
        PRINTF("RESETTHRESHOLDPIN: Threshold PIN not set before, sending NACK.\n");
        sendErrorMessage((char *)"NOPINSET");
    }

    password_correct_flag = false;
}

void setThresholdPIN()
{
    if (!getThresholdSetBeforeFlag())
    {
        PRINTF("SETTHRESHOLDPIN: Threshold PIN set before, setting pin...\n");

        gpio_set_level(THRESHOLD_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(10));
        setThresholdSetBeforeFlag(1);

        PRINTF("SETTHRESHOLDPIN: Threshold PIN set\n");
    }
}
#endif

void readTime()
{
    char buffer[20] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.9.1(%02d:%02d:%02d)%c", 0x02, current_time.hour, current_time.min, current_time.sec, 0x03);
    if (result >= (int)sizeof(buffer))
    {
        PRINTF("READTIME: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"TIMEBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);
    uart_puts_esp(buffer);
    uart_putc_esp(xor_result);
}

void readDate()
{
    char buffer[20] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.9.2(%02d:%02d:%02d)%c", 0x02, current_time.year, current_time.month, current_time.day, 0x03);
    if (result >= (int)sizeof(buffer))
    {
        PRINTF("READDATE: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"DATEBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);
    uart_puts_esp(buffer);
    uart_putc_esp(xor_result);
}

void readSerialNumber()
{
    char buffer[22] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c0.0.0(%s)%c", 0x02, DEVICE_SERIAL_NUMBER, 0x03);
    if (result >= (int)sizeof(buffer))
    {
        PRINTF("READSERIALNUMBER: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"SERIALBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);
    uart_puts_esp(buffer);
    uart_putc_esp(xor_result);
}

void sendLastVRMSXValue(enum ListeningStates vrmsState)
{
    char buffer[20] = {0};
    int result = -1;
    uint8_t xor_result = 0x02;

    switch (vrmsState)
    {
    case ReadLastVRMSMax:
        result = snprintf((char *)buffer, sizeof(buffer), "%c32.7.0(%.2f)%c", 0x02, vrms_max_last, 0x03);
        break;
    case ReadLastVRMSMin:
        result = snprintf((char *)buffer, sizeof(buffer), "%c52.7.0(%.2f)%c", 0x02, vrms_min_last, 0x03);
        break;
    case ReadLastVRMSMean:
        result = snprintf((char *)buffer, sizeof(buffer), "%c72.7.0(%.2f)%c", 0x02, vrms_mean_last, 0x03);
        break;
    default:
        PRINTF("SENDLASTVRMSXVALUE: Unknown state!\n");
        break;
    }

    if (result == -1 || result >= (int)sizeof(buffer))
    {
        PRINTF("SENDLASTVRMSXVALUE: Buffer Overflow or Unknown state! Sending NACK.\n");
        sendErrorMessage((char *)"VRMSBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);
    uart_puts_esp(buffer);
    uart_putc_esp(xor_result);
}

void sendThresholdObis()
{
    char buffer[22] = {0};
    uint8_t xor_result = 0x02;

    int result = snprintf((char *)buffer, sizeof(buffer), "%c96.3.12(%03d)%c", 0x02, getVRMSThresholdValue(), 0x03);
    if (result >= (int)sizeof(buffer))
    {
        PRINTF("SENDTHRESHOLDOBIS: Buffer Overflow! Sending NACK.\n");
        sendErrorMessage((char *)"SERIALBUFFEROVERFLOW");
        return;
    }

    bccGenerate((uint8_t *)buffer, result, &xor_result);
    uart_puts_esp(buffer);
    uart_putc_esp(xor_result);
}
