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

// duration alaninin uc anlami:
#define TH_DURATION_STARTED 0xFFFFu // olay BASLADI, hala acik
#define TH_DURATION_ONGOING 0xFFFEu // olay SURUYOR (ara kayit), hala acik
#define TH_DURATION_MAX 0xFFFDu     // en buyuk gercek sure (dakika) ~45 gun

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
    TH_ACTION_EVENT_STARTED, // olay basladi   -> BASLADI kaydi yazilir
    TH_ACTION_EVENT_ONGOING, // olay uzuyor    -> SURUYOR kaydi yazilir
    TH_ACTION_EVENT_ENDED,   // olay bitti     -> tepe + sure kaydi yazilir
} th_action_t;

typedef struct {
    th_time_t time;    // BASLADI: olayin basi. SURUYOR: o an. BITTI: dusus ani.
    uint16_t vrms_cv;  // BASLADI: tetikleyen deger, digerleri: olay tepesi
    uint16_t duration; // TH_DURATION_STARTED / _ONGOING veya dakika cinsinden sure
} th_record_t;

typedef struct {
    // yapilandirma
    uint16_t enter_cv;          // giris esigi (santivolt)
    uint16_t release_cv;        // cikis esigi (histerezis icin daha dusuk)
    uint16_t enter_windows;     // giris icin ardisik pencere sayisi
    uint16_t exit_windows;      // cikis icin ardisik pencere sayisi
    uint32_t heartbeat_minutes; // uzun olaylarda ara kayit periyodu (0 = kapali)

    // calisma durumu
    bool active;
    bool just_resumed; // acilista devralindi, ilk pencere henuz islenmedi
    uint16_t above_count;
    uint16_t below_count;
    uint16_t peak_cv;
    th_time_t start_time;       // olayin basi
    th_time_t drop_time;        // esigin altina ILK dustugu pencere
    th_time_t last_record_time; // flash'a en son yazilan kaydin zamani
} th_state_t;

// --- Zaman aritmetigi --------------------------------------------------------

// 2000 + year varsayilir (RTC 2 haneli yil veriyor). Sabit bir baslangictan
// itibaren gecen dakika sayisini dondurur; sadece FARK almak icin kullanilir.
uint32_t thTimeToMinutes(const th_time_t *t);

// to - from, dakika. Negatifse (saat geri gitmisse) 0 doner.
uint32_t thMinutesBetween(const th_time_t *from, const th_time_t *to);

// --- Olay mantigi ------------------------------------------------------------

void thEventInit(th_state_t *s, uint16_t enter_cv, uint16_t release_cv,
                 uint16_t enter_windows, uint16_t exit_windows,
                 uint32_t heartbeat_minutes);

// Acilista, flash'ta acik kalmis bir olayi devralir. start_time olayin gercek
// basi, peak_cv o ana kadarki tepe, last_record_time da flash'a en son yazilan
// kaydin zamanidir. Devralmadan sonraki ILK pencere esigin altindaysa olay
// (ariza reset sirasinda gecmis kabul edilerek) last_record_time'da kapatilir.
void thEventResume(th_state_t *s, const th_time_t *start_time, uint16_t peak_cv,
                   const th_time_t *last_record_time);

// Bir pencereyi isler. Donus TH_ACTION_NONE degilse *out doldurulmustur ve
// cagiran tarafin o kaydi flash'a yazmasi beklenir.
th_action_t thEventWindow(th_state_t *s, uint16_t median_cv,
                          const th_time_t *now, th_record_t *out);

// buf yerinde siralanir. n == 0 icin 0 doner.
uint16_t thMedianCv(uint16_t *buf, uint16_t n);

// Volt cinsinden float bir VRMS degerini santivolta cevirir (kirpmali).
uint16_t thVoltsToCv(float volts);

// --- Kayit cozumleme ---------------------------------------------------------
// Ham 16 baytlik kaydi cozer. Yerlesim struct ThresholdData ile AYNI olmali;
// adc.c icindeki _Static_assert'ler bunu derleme zamaninda zorluyor.
//   0-11 : yil/ay/gun/saat/dakika/saniye, her biri 2 ASCII rakam
//   12-13: vrms, santivolt, little-endian
//   14-15: duration, little-endian
// Slot bos veya bozuksa false doner.
bool thDecodeRecord(const uint8_t *raw, th_time_t *time, uint16_t *vrms_cv,
                    uint16_t *duration);

static inline bool thDurationIsOpen(uint16_t duration) {
    return duration == TH_DURATION_STARTED || duration == TH_DURATION_ONGOING;
}

// Halkada geriye dogru yuruyup acik kalmis bir olay arar. fetch(ctx, back, out)
// en son yazilandan `back` kayit onceki ham 16 bayti doldurmali (back = 1 en
// son kayit); slot yoksa false dondurmeli.
typedef bool (*th_record_fetch_fn)(void *ctx, uint16_t back, uint8_t *out);

// Acik olay bulunursa true doner ve start_time / peak_cv / last_record_time
// doldurulur. Zincirde BASLADI kaydi max_back icinde bulunamazsa baslangic
// olarak en son kaydin zamani kullanilir (sure eksik cikar ama sonsuz aramaz).
bool thFindOpenEvent(th_record_fetch_fn fetch, void *ctx, uint16_t max_back,
                     th_time_t *start_time, uint16_t *peak_cv,
                     th_time_t *last_record_time);

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
