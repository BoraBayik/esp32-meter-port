#ifndef THRESHOLD_EVENT_H
#define THRESHOLD_EVENT_H

#include <stdbool.h>
#include <stdint.h>

// Esik olay mantigi -- donanimdan tamamen bagimsizdir (pico SDK / FreeRTOS
// bagimliligi yok) ki host uzerinde test edilebilsin.
//
// Butun gerilimler SANTIVOLT cinsindendir: 1 birim = 0.01 V, 573 = 5.73 V.
// Float yok; esik karsilastirmasi tam sayi uzerinden yapilir.
//
// Mantik pencere basina calisir. Bir pencere = 1 dakika (60 saniyelik VRMS
// orneginin medyani). Amac tek tek ornekleri degil, ARIZA OLAYLARINI kaydetmek:
// "en son ne zaman esik asildi, ne kadar surdu, ne kadar kotuydu".

#define TH_DURATION_ONGOING 0xFFFFu // olay hala suruyor
#define TH_DURATION_MAX 0xFFFEu     // dakika alaninin doyum degeri

typedef struct {
    int16_t year; // 2 haneli (RTC cipinden geldigi gibi): 26 = 2026
    int8_t month;
    int8_t day;
    int8_t hour;
    int8_t min;
    int8_t sec;
} th_time_t;

typedef enum {
    TH_ACTION_NONE = 0,
    TH_ACTION_EVENT_STARTED, // olay basladi   -> "suruyor" kaydi yazilir
    TH_ACTION_EVENT_ONGOING, // olay uzuyor    -> periyodik "suruyor" kaydi
    TH_ACTION_EVENT_ENDED,   // olay bitti     -> tepe + sure kaydi yazilir
} th_action_t;

typedef struct {
    th_time_t time;    // olay basi (STARTED) veya olay sonu (ONGOING/ENDED)
    uint16_t vrms_cv;  // STARTED: tetikleyen deger, digerleri: olay tepesi
    uint16_t duration; // dakika; TH_DURATION_ONGOING = surmekte
} th_record_t;

typedef struct {
    // yapilandirma
    uint16_t enter_cv;          // giris esigi (santivolt)
    uint16_t release_cv;        // cikis esigi (histerezis icin daha dusuk)
    uint16_t enter_windows;     // giris icin ardisik pencere sayisi
    uint16_t exit_windows;      // cikis icin ardisik pencere sayisi
    uint32_t heartbeat_windows; // uzun olaylarda ara kayit periyodu (0 = kapali)

    // calisma durumu
    bool active;
    uint16_t above_count;
    uint16_t below_count;
    uint16_t peak_cv;
    uint32_t minutes;        // olay basindan beri islenen pencere sayisi
    uint32_t last_heartbeat; // son ara kaydin yazildigi dakika
    th_time_t candidate_time;
} th_state_t;

// Durumu sifirlar ve esikleri kurar. release_cv >= enter_cv verilirse cikis
// esigi giris esigine esitlenir (histerezis yok).
void thEventInit(th_state_t *s, uint16_t enter_cv, uint16_t release_cv,
                 uint16_t enter_windows, uint16_t exit_windows,
                 uint32_t heartbeat_windows);

// Bir pencereyi isler. Donus TH_ACTION_NONE degilse *out doldurulmustur ve
// cagiran tarafin o kaydi flash'a yazmasi beklenir.
th_action_t thEventWindow(th_state_t *s, uint16_t median_cv,
                          const th_time_t *now, th_record_t *out);

// buf yerinde siralanir. n == 0 icin 0 doner.
uint16_t thMedianCv(uint16_t *buf, uint16_t n);

// Volt cinsinden float bir VRMS degerini santivolta cevirir (kirpmali).
uint16_t thVoltsToCv(float volts);

// --- Halka tampon aritmetigi -------------------------------------------------
// Kayit alani sabit boyutlu sektorlerden olusan bir HALKA tampondur. Son sektor
// dolunca 0. sektore donulur, o sektor silinip uzerine yazilir; en eski kayitlar
// duser. Bu fonksiyonlar saftir (flash erisimi yok) ki host'ta test edilebilsin.

typedef struct {
    uint16_t sector;         // kaydin yazilacagi sektor
    uint16_t slot_in_sector; // sektor icindeki slot indeksi
    bool sector_changed;     // yeni sektore gecildi (silinip bastan yazilmali)
} th_write_pos_t;

// Sektor icindeki ilk bos slotun bayt offsetini dondurur; sektor doluysa
// sector_size doner. Bos slot = ilk bayti 0xFF (silinmis) veya 0x00.
uint16_t thFindFreeOffset(const uint8_t *sector_data, uint16_t sector_size,
                          uint16_t record_size);

// Bir sonraki kaydin nereye yazilacagini soyler. free_offset, thFindFreeOffset
// sonucudur; sector_size'a esitse sektor dolu demektir ve bir sonraki sektore
// (gerekirse basa donerek) gecilir.
th_write_pos_t thNextWritePos(uint16_t current_sector, uint16_t free_offset,
                              uint16_t sector_size, uint16_t record_size,
                              uint16_t sector_count);

// Yazma konumundan geriye dogru `back` kayit onceki mutlak slot indeksi.
// back = 1 en son yazilan kayittir.
uint16_t thSlotBack(uint16_t write_index, uint16_t back, uint16_t slot_count);

#endif
