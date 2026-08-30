#include "header/spiflash.h"
#include "header/uart.h"
#include "header/project_globals.h"
#include "header/bcc.h"
#include "header/print.h"
#include "header/mutex.h"
#include "esp_partition.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

// dev branch'teki blink/src/spiflash.c dosyasindan uyarlanmistir.

// Partition'i ismiyle bulan yardimci fonksiyon - dev'deki XIP_BASE+offset
// dogrudan pointer erisiminin ESP32 karsiligi. Her cagrida arama yapiyor
// (esp_partition_find_first hafif bir islem, performans sorunu yaratmaz -
// bu fonksiyonlar saniyede degil, dakikada/saatte bir cagriliyor).
static const esp_partition_t *get_partition(const char *label)
{
    const esp_partition_t *p = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, CUSTOM_PARTITION_SUBTYPE, label);
    if (p == NULL)
    {
        PRINTF("SPIFLASH: '%s' partition'i bulunamadi!\n", label);
    }
    return p;
}
// Bu ASAMA 1: sadece saf mantik fonksiyonlari - donanima hic dokunmuyorlar,
// mantik BIREBIR AYNI kaldi (sadece include yollari duzeltildi).

// This function converts the datetime value to char array as characters to
// write flash correctly
void setDateToCharArray(int value, char *array)
{
    // if value is smaller than 10, second character of array will be always 0,
    // first element of array will be value
    if (value < 10)
    {
        array[0] = '0';
        array[1] = value + '0';
    }
    // if value is bigger than 10, two character will be written to char array,
    // first element of array keeps ones place, second element of array keeps
    // tens place
    else
    {
        array[0] = value / 10 + '0';
        array[1] = value % 10 + '0';
    }
}

// this function converts a float value's floating value to uint8_t value.
uint8_t floatDecimalDigitToUint8t(float float_value)
{
    // get floating value of float value and multiply it with 10, so we can get
    // the first digit. We set that value an uint8_t value because we don't want
    // to get rest of the floating digits.
    uint8_t floating_value = (float_value - (float)floor(float_value)) * 10;

    PRINTF("float value before subtraction: %f\n", float_value);
    PRINTF("floating value after floor and uint8_t: %d\n\n", floating_value);

    return floating_value;
}

// This function gets a buffer which includes VRMS values, and calculate the
// max, min and mean values of this buffer and sets the variables.
VRMS_VALUES_RECORD vrmsSetMinMaxMean(float *buffer, uint16_t size)
{
    float buffer_max = buffer[0];
    float buffer_min = buffer[0];
    float buffer_sum = buffer[0];
    VRMS_VALUES_RECORD vrms_values;

    for (uint16_t i = 1; i < size; i++)
    {
        if (buffer[i] > buffer_max)
        {
            buffer_max = buffer[i];
        }

        if (buffer[i] < buffer_min)
        {
            buffer_min = buffer[i];
        }

        buffer_sum += buffer[i];
    }

    vrms_values.vrms_max = (uint8_t)floor(buffer_max);
    vrms_values.vrms_min = (uint8_t)floor(buffer_min);
    vrms_values.vrms_max_dec = floatDecimalDigitToUint8t(buffer_max);
    vrms_values.vrms_min_dec = floatDecimalDigitToUint8t(buffer_min);

    if (size == 0)
    {
        vrms_values.vrms_mean = 0;
        vrms_values.vrms_mean_dec = 0;
    }
    else
    {
        vrms_values.vrms_mean = (uint8_t)(buffer_sum / size);
        vrms_values.vrms_mean_dec = floatDecimalDigitToUint8t(buffer_sum / size);
    }

    PRINTF("buffer max: %f, vrms_max: %d, vrms_max_dec: %d\n", buffer_max,
           vrms_values.vrms_max, vrms_values.vrms_max_dec);
    PRINTF("buffer min: %f, vrms_min: %d, vrms_min_dec: %d\n", buffer_min,
           vrms_values.vrms_min, vrms_values.vrms_min_dec);
    PRINTF("buffer mean: %f, vrms_mean: %d, vrms_mean_dec: %d\n",
           buffer_sum / size, vrms_values.vrms_mean, vrms_values.vrms_mean_dec);

    return vrms_values;
}

float convertVRMSValueToFloat(uint8_t value, uint8_t value_dec)
{
    return value + value_dec / 10.0;
}

// this function converts an array to datetime value
void arrayToDatetime(datetime_t *dt, uint8_t *arr)
{
    dt->year = (arr[0] - '0') * 10 + (arr[1] - '0');
    dt->month = (arr[2] - '0') * 10 + (arr[3] - '0');
    dt->day = (arr[4] - '0') * 10 + (arr[5] - '0');
    dt->hour = (arr[6] - '0') * 10 + (arr[7] - '0');
    dt->min = (arr[8] - '0') * 10 + (arr[9] - '0');
}

// This functon compares two datetime values and return an int value
int datetimeComp(datetime_t *dt1, datetime_t *dt2)
{
    if (dt1->year - dt2->year != 0)
    {
        return dt1->year - dt2->year;
    }
    else if (dt1->month - dt2->month != 0)
    {
        return dt1->month - dt2->month;
    }
    else if (dt1->day - dt2->day != 0)
    {
        return dt1->day - dt2->day;
    }
    else if (dt1->hour - dt2->hour != 0)
    {
        return dt1->hour - dt2->hour;
    }
    else if (dt1->min - dt2->min != 0)
    {
        return dt1->min - dt2->min;
    }
    else if (dt1->sec - dt2->sec != 0)
    {
        return dt1->sec - dt2->sec;
    }

    return 0;
}

// This function copies a datetime value to another datetime value
void datetimeCopy(datetime_t *src, datetime_t *dst)
{
    dst->year = src->year;
    dst->month = src->month;
    dst->day = src->day;
    dst->dotw = src->dotw;
    dst->hour = src->hour;
    dst->min = src->min;
    dst->sec = src->sec;
}

// ============================================================================
// ASAMA 2: gercek flash okuma/yazma
// dev'deki XIP_BASE+offset dogrudan pointer erisimi ve flash_range_erase/
// flash_range_program cagrilari, esp_partition_read/write/erase_range ile
// degistirildi. Mantik (hangi veri nereye/nasil yaziliyor) BIREBIR AYNI.
// ============================================================================

// This function gets the contents like sector data, last records contents from
// flash and sets them to variables.
void getFlashContents()
{
    const esp_partition_t *lp_sector_part = get_partition(PARTITION_LABEL_LP_SECTOR);
    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    const esp_partition_t *threshold_prm_part = get_partition(PARTITION_LABEL_THRESHOLD_PRM);

    if (lp_sector_part == NULL || load_profile_part == NULL || threshold_prm_part == NULL)
    {
        return;
    }

    // get sector count of records
    uint16_t sector_content = 0xFFFF;
    esp_partition_read(lp_sector_part, 0, &sector_content, sizeof(sector_content));
    sector_data = (sector_content == 0xFFFF) ? 0 : sector_content;

    // get record data
    esp_partition_read(load_profile_part, sector_data * FLASH_SECTOR_SIZE, flash_data, FLASH_SECTOR_SIZE);

    // get threshold data
    uint16_t th_buf[2] = {0xFFFF, 0xFFFF};
    esp_partition_read(threshold_prm_part, 0, th_buf, sizeof(th_buf));
    vrms_threshold = (th_buf[0] == 0xFFFF) ? vrms_threshold : th_buf[0];
    th_sector_data = (th_buf[1] == 0xFFFF) ? 0 : th_buf[1];

    // Seri no ARTIK FLASH'TAN OKUNMUYOR: DEVICE_SERIAL_NUMBER makrosu
    // (project_conf.h) her yerde dogrudan kullaniliyor - bkz. uart.c'deki
    // control_serial_number/readSerialNumber. "serial_num" partition'i
    // tabloda duruyor ama artik hic okunmuyor/yazilmiyor.

    PRINTF("GETFLASHCONTENTS: vrms threshold value is: %d\n", vrms_threshold);
    PRINTF("GETFLASHCONTENTS: flash sector is: %d\n", sector_data);
    PRINTF("GETFLASHCONTENTS: threshold sector is: %d\n", th_sector_data);
}

// This function writes current sector data to flash.
void setSectorData(uint16_t sector_value)
{
    const esp_partition_t *lp_sector_part = get_partition(PARTITION_LABEL_LP_SECTOR);
    if (lp_sector_part == NULL)
    {
        return;
    }

    uint8_t buf[4] = {0}; // min 4 byte hizali yazma kisiti
    memcpy(buf, &sector_value, sizeof(sector_value));

    PRINTF("SETSECTORDATA: sector data which is going to be written: %d\n", sector_value);

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("SETSECTORDATA: write flash mutex received\n");
        esp_partition_erase_range(lp_sector_part, 0, FLASH_SECTOR_SIZE);
        esp_partition_write(lp_sector_part, 0, buf, sizeof(buf));
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
    }
}

// This function sets the current time values which are 16 bytes total and
// calculated VRMS values to flash
void setFlashData(VRMS_VALUES_RECORD *vrms_values)
{
    // initialize the variables
    struct FlashData data;
    uint16_t offset = 0;

    // set date values and VRMS values to FlashData struct variable
    setDateToCharArray(current_time.year, data.year);
    setDateToCharArray(current_time.month, data.month);
    setDateToCharArray(current_time.day, data.day);
    setDateToCharArray(current_time.hour, data.hour);
    setDateToCharArray(current_time.min, data.min);
    data.max_volt = vrms_values->vrms_max;
    data.min_volt = vrms_values->vrms_min;
    data.mean_volt = vrms_values->vrms_mean;
    data.max_volt_dec = vrms_values->vrms_max_dec;
    data.min_volt_dec = vrms_values->vrms_min_dec;
    data.mean_volt_dec = vrms_values->vrms_mean_dec;

    if (xSemaphoreTake(xVRMSLastValuesMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        // convert last record vrms values to float
        vrms_max_last = convertVRMSValueToFloat(vrms_values->vrms_max, vrms_values->vrms_max_dec);
        vrms_min_last = convertVRMSValueToFloat(vrms_values->vrms_min, vrms_values->vrms_min_dec);
        vrms_mean_last = convertVRMSValueToFloat(vrms_values->vrms_mean, vrms_values->vrms_mean_dec);

        xSemaphoreGive(xVRMSLastValuesMutex);
    }
    else
    {
        led_blink_pattern(LED_ERROR_CODE_VRMS_VALUES_MUTEX_NOT_TAKEN, false);
        vrms_max_last = 0.0;
        vrms_min_last = 0.0;
        vrms_mean_last = 0.0;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("SETFLASHDATA: offset loop mutex received\n");
        // find the last offset of flash records and write current values to
        // last offset of flash_data buffer (RAM'deki kopya - flash'a yazma
        // isini SPIWriteToFlash yapiyor)
        uint8_t *flash_data_bytes = (uint8_t *)flash_data;
        for (offset = 0; offset < FLASH_SECTOR_SIZE; offset += FLASH_RECORD_SIZE)
        {
            if (flash_data_bytes[offset] == '\0' || flash_data_bytes[offset] == 0xff)
            {
                if (offset == 0)
                {
                    PRINTF("SETFLASHDATA: last record is not found.\n");
                }
                else
                {
                    PRINTF("SETFLASHDATA: last record is start in %d offset\n", offset - 16);
                }

                flash_data[offset / FLASH_RECORD_SIZE] = data;

                PRINTF("SETFLASHDATA: record saved to offset: %d. used %d/%d of sector.\n",
                       offset, offset + 16, FLASH_SECTOR_SIZE);

                break;
            }
        }

        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }

    // if offset value is equals or bigger than FLASH_SECTOR_SIZE, (4096 bytes)
    // it means current sector is full and program should write new values to
    // next sector
    if (offset >= FLASH_SECTOR_SIZE)
    {
        PRINTF("SETFLASHDATA: offset value is equals to sector size. Current sector data is: %d. Sector is changing...\n", sector_data);

        // if current sector is last sector of flash, sector data will be 0 and
        // the program will start to write new records to beginning of the flash
        // record offset
        if (sector_data == FLASH_LOAD_PROFILE_AREA_TOTAL_SECTOR_COUNT - 1)
        {
            sector_data = 0;
        }
        else
        {
            sector_data++;
        }

        PRINTF("SETFLASHDATA: new sector value is: %d\n", sector_data);

        // reset variables and call setSectorData()
        memset(flash_data, 0, FLASH_SECTOR_SIZE);
        flash_data[0] = data;
        setSectorData(sector_data);

        PRINTF("SETFLASHDATA: Sector changing written to flash.\n");
    }
}

// This function writes flash_data content to flash area
void SPIWriteToFlash(VRMS_VALUES_RECORD *vrms_values)
{
    PRINTF("SPIWRITETOFLASH: Setting flash data...\n");
    setFlashData(vrms_values);

    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    if (load_profile_part == NULL)
    {
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("SPIWRITETOFLASH: write flash mutex received\n");
        esp_partition_erase_range(load_profile_part, sector_data * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
        esp_partition_write(load_profile_part, sector_data * FLASH_SECTOR_SIZE, flash_data, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
    }
}

// ============================================================================
// ASAMA 3: load profile / sektor kontrol
// ============================================================================
//
// ONEMLI DUZELTME: Once "P.01 ile degisken tarih araligi cekme mekanizmasi
// olu kod, hic kullanilmiyor" diye not dusulmustu - bu YANLIS cikti (o not
// `master` branch icin gecerliydi). `dev`'in gercek uart.c'sinde
// {"P.01", Reading, CMD_TYPE_ANY} satiri var - yani bu mekanizma GERCEKTEN
// kullaniliyor, hem tarihli hem tarihsiz istekte. Bu yuzden asagidaki tum
// tarih araligi mantigi (get_record_indexes, parse_load_profile_dates vs.)
// SADIK sekilde portlandi, atlanmadi.
//
// dev'deki XIP_BASE+FLASH_LOAD_PROFILE_RECORD_ADDR dogrudan pointer erisimi,
// esp_partition_mmap() ile degistirildi - bu, partition'in tamamini (12
// sektor, 48kB) tek bir salt-okunur bellek bolgesi gibi gormemizi sagliyor,
// XIP_BASE+offset'in ESP32 karsiligi.

// This function checks if a datetime value is empty (tum alanlari 0)
static uint8_t is_datetime_empty(datetime_t *dt)
{
    if (dt->year == 0 && dt->month == 0 && dt->day == 0 && dt->hour == 0 && dt->min == 0 && dt->sec == 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// This function validates a datetime value's fields are within sensible ranges
static bool check_datetime_format(datetime_t *dt)
{
    if (dt->year < 0 || dt->year > 99)
    {
        return false;
    }
    if (dt->month < 0 || dt->month > 12)
    {
        return false;
    }
    if (dt->day < 0 || dt->day > 31)
    {
        return false;
    }
    if (dt->hour < 0 || dt->hour > 23)
    {
        return false;
    }
    if (dt->min < 0 || dt->min > 59)
    {
        return false;
    }

    return true;
}

// This function extracts a raw date substring (between two pointers) into a
// clean digit-only array (separators like '-', ',', ':' stripped out)
static void add_date_to_buffer(char *start_ptr, char *end_ptr, uint8_t *date_arr)
{
    size_t date_length = end_ptr - start_ptr - 1;
    uint8_t date_idx = 0;

    if (date_length < 10 && date_length > 14)
    {
        PRINTF("Date length is wrong: %zu\n", date_length);
        return;
    }

    start_ptr++;

    PRINTF("Date length: %zu\n", date_length);
    PRINTF("Date string: %.*s\n", (int)date_length, start_ptr);
    PRINTF("\n");

    for (uint8_t i = 0; i < date_length; i++)
    {
        if (start_ptr[i] == '-' || start_ptr[i] == ',' || start_ptr[i] == ':')
        {
            continue;
        }
        date_arr[date_idx++] = start_ptr[i];
    }

    date_arr[date_idx] = '\0';
}

// This function parses a "P.01(start;end)" style buffer into start/end datetime
// values. Return value tells which parts were actually present:
// 0=format error, 1=both empty, 2=only end given, 3=only start given, 4=both given
static uint8_t parse_load_profile_dates(uint8_t *buf, datetime_t *dt_start, datetime_t *dt_end)
{
    char *lp_start_ptr = NULL;
    char *lp_end_ptr = NULL;
    char *lp_date_seperator_ptr = NULL;
    uint8_t start_date_arr[16];
    uint8_t end_date_arr[16];

    memset(start_date_arr, 0, sizeof(start_date_arr));
    memset(end_date_arr, 0, sizeof(end_date_arr));

    lp_start_ptr = strchr((char *)buf, '(');
    lp_end_ptr = strchr((char *)buf, ')');
    lp_date_seperator_ptr = strchr((char *)buf, ';');

    if (lp_start_ptr == NULL || lp_end_ptr == NULL || lp_date_seperator_ptr == NULL || lp_end_ptr <= lp_start_ptr)
    {
        return 0;
    }
    else if (lp_start_ptr + 1 == lp_date_seperator_ptr && lp_date_seperator_ptr + 1 == lp_end_ptr)
    {
        return 1;
    }
    else if (lp_start_ptr + 1 == lp_date_seperator_ptr && lp_date_seperator_ptr + 1 != lp_end_ptr)
    {
        add_date_to_buffer(lp_date_seperator_ptr, lp_end_ptr, end_date_arr);
        arrayToDatetime(dt_end, end_date_arr);
        return 2;
    }
    else if (lp_start_ptr + 1 != lp_date_seperator_ptr && lp_date_seperator_ptr + 1 == lp_end_ptr)
    {
        add_date_to_buffer(lp_start_ptr, lp_date_seperator_ptr, start_date_arr);
        arrayToDatetime(dt_start, start_date_arr);

        return 3;
    }

    add_date_to_buffer(lp_start_ptr, lp_date_seperator_ptr, start_date_arr);
    add_date_to_buffer(lp_date_seperator_ptr, lp_end_ptr, end_date_arr);

    arrayToDatetime(dt_start, start_date_arr);
    arrayToDatetime(dt_end, end_date_arr);

    PRINTF("Parsed Start Date: %s\n", start_date_arr);
    PRINTF("Parsed End Date: %s\n", end_date_arr);

    return 4;
}

// This function checks if a 16-byte flash record's datetime falls within [dt_start, dt_end]
static bool is_record_between_date_values(uint8_t *record_ptr, datetime_t *dt_start, datetime_t *dt_end)
{
    datetime_t record_dt = {0};
    arrayToDatetime(&record_dt, record_ptr);

    if (datetimeComp(&record_dt, dt_start) < 0)
    {
        return false;
    }

    if (datetimeComp(&record_dt, dt_end) > 0)
    {
        return false;
    }

    return true;
}

// This function scans the whole load_profile partition (12 sektor, 48kB) and
// finds the byte-offsets of the first/last record that fall within the
// requested date range (or the whole range, if dates are empty)
static uint8_t get_record_indexes(int64_t *start, int64_t *end, datetime_t *dt_start, datetime_t *dt_end)
{
    datetime_t start_dt_local = {0};
    datetime_t end_dt_local = {0};
    uint8_t start_dt_empty_flag = 0;
    uint8_t end_dt_empty_flag = 0;

    memcpy(&start_dt_local, dt_start, sizeof(datetime_t));
    memcpy(&end_dt_local, dt_end, sizeof(datetime_t));

    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    if (load_profile_part == NULL)
    {
        return 0;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        const void *mmap_ptr = NULL;
        esp_partition_mmap_handle_t mmap_handle;
        if (esp_partition_mmap(load_profile_part, 0, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle) != ESP_OK)
        {
            PRINTF("GETRECORDINDEXES: mmap failed!\n");
            xSemaphoreGive(xFlashMutex);
            return 0;
        }
        const uint8_t *flash_records_ptr = (const uint8_t *)mmap_ptr;

        if (is_datetime_empty(dt_start))
        {
            PRINTF("GETRECORDINDEXES: start datetime is empty, setting to first record datetime\n");
            arrayToDatetime(dt_start, (uint8_t *)&flash_records_ptr[0]);
            start_dt_empty_flag = 1;
        }
        if (is_datetime_empty(dt_end))
        {
            PRINTF("GETRECORDINDEXES: end datetime is empty, setting to first record datetime\n");
            arrayToDatetime(dt_end, (uint8_t *)&flash_records_ptr[0]);
            end_dt_empty_flag = 1;
        }

        for (uint32_t i = 0; i < FLASH_LOAD_PROFILE_RECORD_AREA_SIZE; i += FLASH_RECORD_SIZE)
        {
            // if current index is empty, continue
            if (flash_records_ptr[i] == 0xFF || flash_records_ptr[i] == 0x00)
            {
                continue;
            }

            // if current index is not empty, set datetime to current index record
            datetime_t recurrent_time = {0};
            arrayToDatetime(&recurrent_time, (uint8_t *)&flash_records_ptr[i]);

            if (start_dt_empty_flag)
            {
                if (datetimeComp(&recurrent_time, dt_start) <= 0)
                {
                    *start = i;
                    datetimeCopy(&recurrent_time, &start_dt_local);
                }
            }
            else
            {
                // if current record datetime is bigger than start datetime and start index is not set, this is the start index
                if (datetimeComp(&recurrent_time, dt_start) >= 0)
                {
                    if (is_datetime_empty(&start_dt_local))
                    {
                        *start = i;
                        datetimeCopy(&recurrent_time, &start_dt_local);
                    }
                    else if (*start == -1 || (datetimeComp(&recurrent_time, &start_dt_local) < 0))
                    {
                        *start = i;
                        datetimeCopy(&recurrent_time, &start_dt_local);
                    }
                }
            }

            if (end_dt_empty_flag)
            {
                if (datetimeComp(&recurrent_time, dt_end) >= 0)
                {
                    *end = i;
                    datetimeCopy(&recurrent_time, &end_dt_local);
                }
            }
            else
            {
                // if current record datetime is smaller than end datetime and end index is not set, this is the end index
                if (datetimeComp(&recurrent_time, dt_end) <= 0)
                {
                    if (is_datetime_empty(&end_dt_local))
                    {
                        *end = i;
                        datetimeCopy(&recurrent_time, &end_dt_local);
                    }
                    else if (*end == -1 || (datetimeComp(&recurrent_time, &end_dt_local) > 0))
                    {
                        *end = i;
                        datetimeCopy(&recurrent_time, &end_dt_local);
                    }
                }
            }
        }

        esp_partition_munmap(mmap_handle);
        xSemaphoreGive(xFlashMutex);
        return 1;
    }
    else
    {
        PRINTF("GETRECORDINDEXES: Could not take flash mutex!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return 0;
    }
}

// This function searches the requested data in flash by starting from flash
// record beginning offset, collects data from flash and sends it to UART to
// show load profile content
void send_load_profile_records(uint8_t *buf)
{
    datetime_t dt_start;
    datetime_t dt_end;
    int64_t start_index = -1;
    int64_t end_index = -1;
    char load_profile_line[48] = {0};

    char year[3] = {0};
    char month[3] = {0};
    char day[3] = {0};
    char hour[3] = {0};
    char minute[3] = {0};
    uint8_t max = 0;
    uint8_t max_dec = 0;
    uint8_t min = 0;
    uint8_t min_dec = 0;
    uint8_t mean = 0;
    uint8_t mean_dec = 0;

    memset(&dt_start, 0, sizeof(datetime_t));
    memset(&dt_end, 0, sizeof(datetime_t));

    uint8_t parse_result = parse_load_profile_dates(buf, &dt_start, &dt_end);

    if (!check_datetime_format(&dt_start) || !check_datetime_format(&dt_end) || parse_result == 0)
    {
        PRINTF("SEARCHDATAINFLASH: Date format is wrong.\n");
        sendErrorMessage((char *)"LPDATEFORMAT");
        return;
    }

    uint8_t result = get_record_indexes(&start_index, &end_index, &dt_start, &dt_end);

    if (result == 0 || (start_index == -1 && end_index == -1))
    {
        PRINTF("SEARCHDATAINFLASH: Error occurred while getting record indexes.\n");
        sendErrorMessage((char *)"LPGETRECIDXERR");
        return;
    }
    PRINTF("SEARCHDATAINFLASH: Start index is: %lld\n", start_index);
    PRINTF("SEARCHDATAINFLASH: End index is: %lld\n", end_index);

    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    if (load_profile_part == NULL)
    {
        sendErrorMessage((char *)"LPPARTNOTFOUND");
        return;
    }

    // if there start and end index are set, there are records between these
    // times so send them to UART
    if (start_index >= 0 && end_index >= 0)
    {
        PRINTF("SEARCHDATAINFLASH: Generating messages...\n");

        // initialize the variables
        uint8_t xor_result = 0x00;
        uint32_t start_addr = start_index;
        uint32_t end_addr = start_index <= end_index
                                 ? end_index
                                 : (uint32_t)(FLASH_LOAD_PROFILE_RECORD_AREA_SIZE - FLASH_RECORD_SIZE);
        int snprintf_result;
        // Mutex ust uste alinamazsa donguden CIKILMALI. Eskiden `continue`
        // adresi ilerletmeden tekrar deniyordu; tek cikis yolu watchdog'un
        // cihazi resetlemesiydi.
        uint8_t mutex_fail_count = 0;

        // send STX character
        uint8_t stx_byte = STX;
        uart_write_bytes(UART_PORT_NUM, (const char *)&stx_byte, 1);

        while (start_addr <= end_addr)
        {
            // 300 baud'da bu dongu dakikalarca surebiliyor; UART gorevi mesaj
            // dongusune donemedigi icin watchdog'u burada besliyoruz.
            uartTaskHeartbeat();

            if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
            {
                mutex_fail_count = 0;
                const void *mmap_ptr = NULL;
                esp_partition_mmap_handle_t mmap_handle;
                if (esp_partition_mmap(load_profile_part, 0, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle) != ESP_OK)
                {
                    PRINTF("SEARCHDATAINFLASH: mmap failed!\n");
                    xSemaphoreGive(xFlashMutex);
                    break;
                }
                const uint8_t *flash_start_content = (const uint8_t *)mmap_ptr;

                if (flash_start_content[start_addr] == 0xFF ||
                    flash_start_content[start_addr] == 0x00 ||
                    (is_record_between_date_values((uint8_t *)&flash_start_content[start_addr], &dt_start, &dt_end) == false && parse_result == 4))
                {
                    esp_partition_munmap(mmap_handle);
                    start_addr += FLASH_RECORD_SIZE;
                    xSemaphoreGive(xFlashMutex);
                    continue;
                }
                PRINTF("SEARCHDATAINFLASH: set data mutex received\n");

                // set char arrays to initialize a string for record
                snprintf(year, sizeof(year), "%c%c", flash_start_content[start_addr], flash_start_content[start_addr + 1]);
                snprintf(month, sizeof(month), "%c%c", flash_start_content[start_addr + 2], flash_start_content[start_addr + 3]);
                snprintf(day, sizeof(day), "%c%c", flash_start_content[start_addr + 4], flash_start_content[start_addr + 5]);
                snprintf(hour, sizeof(hour), "%c%c", flash_start_content[start_addr + 6], flash_start_content[start_addr + 7]);
                snprintf(minute, sizeof(minute), "%c%c", flash_start_content[start_addr + 8], flash_start_content[start_addr + 9]);
                max = flash_start_content[start_addr + 10];
                max_dec = flash_start_content[start_addr + 11];
                min = flash_start_content[start_addr + 12];
                min_dec = flash_start_content[start_addr + 13];
                mean = flash_start_content[start_addr + 14];
                mean_dec = flash_start_content[start_addr + 15];

                esp_partition_munmap(mmap_handle);
                xSemaphoreGive(xFlashMutex);
            }
            else
            {
                led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);

                if (++mutex_fail_count >= LP_MUTEX_MAX_RETRY)
                {
                    PRINTF("SEARCHDATAINFLASH: flash mutex %d kez alinamadi, "
                           "okuma iptal edildi.\n", LP_MUTEX_MAX_RETRY);
                    sendErrorMessage((char *)"FLASHMUTEXERR");
                    return;
                }

                continue;
            }

            snprintf_result = snprintf(load_profile_line, sizeof(load_profile_line),
                                        "(%s-%s-%s,%s:%s)(%03d.%d,%03d.%d,%03d.%d)\r\n",
                                        year, month, day, hour, minute, min, min_dec, max, max_dec, mean, mean_dec);
            bccGenerate((uint8_t *)load_profile_line, 40, &xor_result);
            PRINTF("SEARCHDATAINFLASH: message to send: %s\n", load_profile_line);

            if (snprintf_result >= (int)sizeof(load_profile_line))
            {
                PRINTF("SEARCHDATAINFLASH: Buffer Overflow! Sending NACK.\n");
                sendErrorMessage((char *)"LPBUFFEROVERFLOW");
            }
            else
            {
                uart_write_bytes(UART_PORT_NUM, load_profile_line, strlen(load_profile_line));
            }

            // if start address equals to end address, it means there is just
            // one record to send or this record is the last record to send
            if (start_addr == end_addr && start_index > end_index && start_addr == FLASH_LOAD_PROFILE_RECORD_AREA_SIZE - FLASH_RECORD_SIZE)
            {
                start_addr = 0;
                end_addr = end_index;
            }

            vTaskDelay(pdMS_TO_TICKS(15));
            start_addr += FLASH_RECORD_SIZE;
        }

        uint8_t cr_byte = '\r';
        uart_write_bytes(UART_PORT_NUM, (const char *)&cr_byte, 1);
        xor_result ^= '\r';

        uint8_t etx_byte = ETX;
        uart_write_bytes(UART_PORT_NUM, (const char *)&etx_byte, 1);
        xor_result ^= ETX;

        uart_write_bytes(UART_PORT_NUM, (const char *)&xor_result, 1);

        return;
    }

    PRINTF("SEARCHDATAINFLASH: data not found.\n");
    sendErrorMessage((char *)"NODATAFOUND");
}

// ============================================================================
// YENI (BLE tarih-aralikli sorgu icin, kullanicinin istegiyle eklendi) -
// send_load_profile_records() ile AYNI arama/filtreleme mantigini (yukaridaki
// static get_record_indexes/is_record_between_date_values) kullanir, ama
// RS485/BCC cerceveleme yerine duz bir metin tamponuna yaziyor.
// ============================================================================

// dt_start/dt_end araligindaki load profile kayitlarini
// "tarih,saat,min,max,ortalama;..." formatinda out_buf'a yazar - mantik
// send_load_profile_records()'un ana donguyle BIREBIR AYNI (wraparound dahil).
void getLoadProfileRecordsAsText(datetime_t *dt_start, datetime_t *dt_end, char *out_buf, size_t out_buf_size)
{
    int64_t start_index = -1;
    int64_t end_index = -1;
    size_t pos = 0;
    out_buf[0] = '\0';

    if (!check_datetime_format(dt_start) || !check_datetime_format(dt_end))
    {
        snprintf(out_buf, out_buf_size, "tarih formati gecersiz");
        return;
    }

    uint8_t result = get_record_indexes(&start_index, &end_index, dt_start, dt_end);
    if (result == 0 || (start_index == -1 && end_index == -1))
    {
        snprintf(out_buf, out_buf_size, "veri bulunamadi");
        return;
    }

    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    if (load_profile_part == NULL)
    {
        snprintf(out_buf, out_buf_size, "partition bulunamadi");
        return;
    }

    uint32_t start_addr = (uint32_t)start_index;
    uint32_t end_addr = start_index <= end_index
                             ? (uint32_t)end_index
                             : (uint32_t)(FLASH_LOAD_PROFILE_RECORD_AREA_SIZE - FLASH_RECORD_SIZE);

    while (start_addr <= end_addr)
    {
        if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
        {
            const void *mmap_ptr = NULL;
            esp_partition_mmap_handle_t mmap_handle;
            if (esp_partition_mmap(load_profile_part, 0, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle) != ESP_OK)
            {
                xSemaphoreGive(xFlashMutex);
                break;
            }
            const uint8_t *flash_start_content = (const uint8_t *)mmap_ptr;

            if (flash_start_content[start_addr] == 0xFF ||
                flash_start_content[start_addr] == 0x00 ||
                !is_record_between_date_values((uint8_t *)&flash_start_content[start_addr], dt_start, dt_end))
            {
                esp_partition_munmap(mmap_handle);
                xSemaphoreGive(xFlashMutex);
                start_addr += FLASH_RECORD_SIZE;
                continue;
            }

            char year[3], month[3], day[3], hour[3], minute[3];
            snprintf(year, sizeof(year), "%c%c", flash_start_content[start_addr], flash_start_content[start_addr + 1]);
            snprintf(month, sizeof(month), "%c%c", flash_start_content[start_addr + 2], flash_start_content[start_addr + 3]);
            snprintf(day, sizeof(day), "%c%c", flash_start_content[start_addr + 4], flash_start_content[start_addr + 5]);
            snprintf(hour, sizeof(hour), "%c%c", flash_start_content[start_addr + 6], flash_start_content[start_addr + 7]);
            snprintf(minute, sizeof(minute), "%c%c", flash_start_content[start_addr + 8], flash_start_content[start_addr + 9]);
            uint8_t max_v = flash_start_content[start_addr + 10];
            uint8_t max_dec = flash_start_content[start_addr + 11];
            uint8_t min_v = flash_start_content[start_addr + 12];
            uint8_t min_dec = flash_start_content[start_addr + 13];
            uint8_t mean_v = flash_start_content[start_addr + 14];
            uint8_t mean_dec = flash_start_content[start_addr + 15];

            esp_partition_munmap(mmap_handle);
            xSemaphoreGive(xFlashMutex);

            int n = snprintf(out_buf + pos, out_buf_size - pos, "%s-%s-%s,%s:%s,%03d.%d,%03d.%d,%03d.%d;",
                              year, month, day, hour, minute, min_v, min_dec, max_v, max_dec, mean_v, mean_dec);
            if (n > 0 && (size_t)n < out_buf_size - pos)
            {
                pos += (size_t)n;
            }
            else
            {
                break; // tampon doldu
            }
        }
        else
        {
            led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
            break;
        }

        // send_load_profile_records()'teki AYNI dairesel-tampon (wraparound) mantigi
        if (start_addr == end_addr && start_index > end_index && start_addr == FLASH_LOAD_PROFILE_RECORD_AREA_SIZE - FLASH_RECORD_SIZE)
        {
            start_addr = 0;
            end_addr = (uint32_t)end_index;
        }

        start_addr += FLASH_RECORD_SIZE;
    }

    if (pos == 0)
    {
        snprintf(out_buf, out_buf_size, "veri bulunamadi");
    }
}

// flash'taki TUM load profile kayitlarinin FARKLI (distinct) tarihlerini
// "YY-MM-DD,YY-MM-DD,..." formatinda (artan sirada) out_buf'a yazar.
void getLoadProfileAvailableDates(char *out_buf, size_t out_buf_size)
{
    static char seen_dates[64][7]; // "YYMMDD\0" - 64 farkli tarih yeterli buyuklukte pay
    int seen_count = 0;
    size_t pos = 0;
    out_buf[0] = '\0';

    const esp_partition_t *load_profile_part = get_partition(PARTITION_LABEL_LOAD_PROFILE);
    if (load_profile_part == NULL)
    {
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
        return;
    }

    const void *mmap_ptr = NULL;
    esp_partition_mmap_handle_t mmap_handle;
    if (esp_partition_mmap(load_profile_part, 0, FLASH_LOAD_PROFILE_RECORD_AREA_SIZE, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle) != ESP_OK)
    {
        xSemaphoreGive(xFlashMutex);
        return;
    }
    const uint8_t *flash_records_ptr = (const uint8_t *)mmap_ptr;

    for (uint32_t i = 0; i < FLASH_LOAD_PROFILE_RECORD_AREA_SIZE; i += FLASH_RECORD_SIZE)
    {
        if (flash_records_ptr[i] == 0xFF || flash_records_ptr[i] == 0x00)
        {
            continue;
        }

        char date_key[7];
        snprintf(date_key, sizeof(date_key), "%c%c%c%c%c%c",
                 flash_records_ptr[i], flash_records_ptr[i + 1],
                 flash_records_ptr[i + 2], flash_records_ptr[i + 3],
                 flash_records_ptr[i + 4], flash_records_ptr[i + 5]);

        bool already_seen = false;
        for (int j = 0; j < seen_count; j++)
        {
            if (strcmp(seen_dates[j], date_key) == 0)
            {
                already_seen = true;
                break;
            }
        }
        if (!already_seen && seen_count < 64)
        {
            strcpy(seen_dates[seen_count], date_key);
            seen_count++;
        }
    }

    esp_partition_munmap(mmap_handle);
    xSemaphoreGive(xFlashMutex);

    // kucuk N icin basit bubble sort yeterli (artan siraya koy)
    for (int a = 0; a < seen_count - 1; a++)
    {
        for (int b = 0; b < seen_count - 1 - a; b++)
        {
            if (strcmp(seen_dates[b], seen_dates[b + 1]) > 0)
            {
                char tmp[7];
                strcpy(tmp, seen_dates[b]);
                strcpy(seen_dates[b], seen_dates[b + 1]);
                strcpy(seen_dates[b + 1], tmp);
            }
        }
    }

    for (int j = 0; j < seen_count; j++)
    {
        int n = snprintf(out_buf + pos, out_buf_size - pos, "%c%c-%c%c-%c%c,",
                          seen_dates[j][0], seen_dates[j][1], seen_dates[j][2],
                          seen_dates[j][3], seen_dates[j][4], seen_dates[j][5]);
        if (n > 0 && (size_t)n < out_buf_size - pos)
        {
            pos += (size_t)n;
        }
        else
        {
            break;
        }
    }
}

// This function checks the "lp_sector" partition on first boot - if it has
// never been written (still 0xFFFF/erased), initialize it to 0
void checkSectorContent()
{
    const esp_partition_t *lp_sector_part = get_partition(PARTITION_LABEL_LP_SECTOR);
    if (lp_sector_part == NULL)
    {
        return;
    }

    uint16_t sector_content = 0xFFFF;
    esp_partition_read(lp_sector_part, 0, &sector_content, sizeof(sector_content));

    if (sector_content == 0xFFFF)
    {
        PRINTF("CHECKSECTORCONTENT: sector area is empty. Sector content is going to set 0.\n");
        uint16_t buf[2] = {0, 0};
        esp_partition_erase_range(lp_sector_part, 0, FLASH_SECTOR_SIZE);
        esp_partition_write(lp_sector_part, 0, buf, sizeof(buf));
    }
}

// This function checks the "threshold_prm" partition on first boot - if the
// threshold value or the threshold-records-sector value has never been
// written (0xFFFF/erased), initialize it to a sensible default
void checkThresholdContent()
{
    const esp_partition_t *threshold_prm_part = get_partition(PARTITION_LABEL_THRESHOLD_PRM);
    if (threshold_prm_part == NULL)
    {
        return;
    }

    uint16_t th_buf[2] = {0xFFFF, 0xFFFF};
    esp_partition_read(threshold_prm_part, 0, th_buf, sizeof(th_buf));

    bool needs_write = false;

    // Threshold value control
    if (th_buf[0] == 0xFFFF)
    {
        PRINTF("threshold value is empty, setting to 5 as default...\n");
        th_buf[0] = vrms_threshold;
        needs_write = true;
    }
    // Threshold Records Sector control
    if (th_buf[1] == 0xFFFF)
    {
        PRINTF("threshold record's sector value is empty, setting to 0 as default...\n");
        th_buf[1] = 0;
        needs_write = true;
    }

    if (needs_write)
    {
        esp_partition_erase_range(threshold_prm_part, 0, FLASH_SECTOR_SIZE);
        esp_partition_write(threshold_prm_part, 0, th_buf, sizeof(th_buf));
    }
}

// This function writes the given threshold-records-sector value to the
// "threshold_prm" partition (alongside the current threshold value)
void updateThresholdSector(uint16_t sector_val)
{
    const esp_partition_t *threshold_prm_part = get_partition(PARTITION_LABEL_THRESHOLD_PRM);
    if (threshold_prm_part == NULL)
    {
        return;
    }

    uint16_t th_buf[2];
    th_buf[0] = getVRMSThresholdValue();
    th_buf[1] = sector_val;

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("UPDATETHRESHOLDSECTOR: write flash mutex received\n");
        esp_partition_erase_range(threshold_prm_part, 0, FLASH_SECTOR_SIZE);
        esp_partition_write(threshold_prm_part, 0, th_buf, sizeof(th_buf));
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
    }
}

// Bu, dev'deki adc.c/writeThresholdRecord() fonksiyonunun kendi icinde
// dogrudan yaptigi flash_range_erase/flash_range_program cagrisinin ESP32
// karsiligi - th_flash_buf'in TAMAMINI (bir sektor, 4096 byte) "threshold_rec"
// partition'inin ilgili sektorune yazar. adc.c'den cagrilir.
void writeThresholdRecordsSectorToFlash(uint16_t sector_val)
{
    const esp_partition_t *threshold_rec_part = get_partition(PARTITION_LABEL_THRESHOLD_REC);
    if (threshold_rec_part == NULL)
    {
        return;
    }

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        PRINTF("WRITETHRESHOLDRECORDSSECTOR: write flash mutex received\n");
        esp_partition_erase_range(threshold_rec_part, sector_val * FLASH_SECTOR_SIZE, FLASH_SECTOR_SIZE);
        esp_partition_write(threshold_rec_part, sector_val * FLASH_SECTOR_SIZE, th_flash_buf, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        PRINTF("MUTEX CANNOT RECEIVED!\n");
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
    }
}

// ============================================================================
// ASAMA 4: reset kaydi / ani degisim kaydi
// ============================================================================

// ⚠️ addSerialNumber() KALDIRILDI. Eskiden seri no ilk acilista "serial_num"
// partition'ina yazilir, sonraki her acilista oradan RAM'e okunurdu. Sorun:
// fonksiyon sadece alan BOS (0xFF) iken yaziyordu, yani bir kez yazildiktan
// sonra DEVICE_SERIAL_NUMBER makrosu degistirilip yeni firmware yuklense bile
// cihaz eski seri numarayi kullanmaya devam ediyordu (degistirmek icin o
// sektoru elle silmek gerekiyordu).
//
// Yeni durum: seri no artik SADECE DEVICE_SERIAL_NUMBER makrosundan geliyor,
// flash'a hic dokunulmuyor - makro degisip firmware yuklendiginde cihazin
// seri numarasi da degisiyor. "serial_num" partition'i partitions.csv'de
// BILEREK duruyor (silinseydi arkasindaki butun partition'larin offset'leri
// kayardi); artik okunmuyor da yazilmiyor da, bos duruyor.

// This function appends the current date/time as a new "reset/boot" record
// (12 kayitlik dongusel arsiv, dolunca basa saradan basliyor - dev'deki ile
// birebir ayni mantik)
void setProgramStartDate(datetime_t *ct)
{
    const esp_partition_t *reset_dates_part = get_partition(PARTITION_LABEL_RESET_DATES);
    if (reset_dates_part == NULL)
    {
        return;
    }

    uint8_t current_time_buffer[16] = {0};
    uint8_t flash_reset_count_buffer[FLASH_SECTOR_SIZE];
    uint16_t offset = 0;

    esp_partition_read(reset_dates_part, 0, flash_reset_count_buffer, FLASH_SECTOR_SIZE);

    setDateToCharArray(ct->year, (char *)current_time_buffer);
    setDateToCharArray(ct->month, (char *)current_time_buffer + 2);
    setDateToCharArray(ct->day, (char *)current_time_buffer + 4);
    setDateToCharArray(ct->hour, (char *)current_time_buffer + 6);
    setDateToCharArray(ct->min, (char *)current_time_buffer + 8);
    setDateToCharArray(ct->sec, (char *)current_time_buffer + 10);

    current_time_buffer[12] = 0x7F;
    current_time_buffer[13] = 0x7F;
    current_time_buffer[14] = 0x7F;
    current_time_buffer[15] = 0x7F;

    PRINTF("SETPROGRAMSTARTDATE: Program start date is set to: ");
    printBufferHex(current_time_buffer, 10);
    PRINTF("\n");

    for (uint16_t i = 0; i < FLASH_SECTOR_SIZE; i += 16)
    {
        if (flash_reset_count_buffer[offset] == 0xFF || flash_reset_count_buffer[offset] == 0x00)
        {
            break;
        }

        offset += 16;
    }

    if (offset >= FLASH_SECTOR_SIZE)
    {
        memset(flash_reset_count_buffer, 0, FLASH_SECTOR_SIZE);
        offset = 0;
    }

    memcpy(flash_reset_count_buffer + offset, current_time_buffer, sizeof(current_time_buffer));

    if (xSemaphoreTake(xFlashMutex, pdMS_TO_TICKS(250)) == pdTRUE)
    {
        esp_partition_erase_range(reset_dates_part, 0, FLASH_SECTOR_SIZE);
        esp_partition_write(reset_dates_part, 0, flash_reset_count_buffer, FLASH_SECTOR_SIZE);
        xSemaphoreGive(xFlashMutex);
    }
    else
    {
        led_blink_pattern(LED_ERROR_CODE_FLASH_MUTEX_NOT_TAKEN, false);
    }
}

// ⚠️ writeSuddenAmplitudeChangeRecordToFlash: `dev` branch'te bu fonksiyon
// spiflash.h'de DECLARE edilmis ama HICBIR YERDE implement edilmemis -
// spiflash.c/adc.c/uart.c'nin hicbirinde govdesi yok. main.c'de
// `#if CONF_SUDDEN_AMPLITUDE_CHANGE_ENABLED` (0, kapali) altinda cagriliyor,
// bu yuzden dev'de hic link hatasi vermemis - ozellik hic acilmadigi icin
// eksik oldugu fark edilmemis. Bizde de ayni bayrak kapali (0), bu yuzden
// simdilik implement ETMIYORUZ (portlanacak gercek bir kod yok, dev'in
// kendisinde de yok). Ozellik ileride acilirsa, once dev tarafinda da
// gercekten yazilmasi gerekecek.
