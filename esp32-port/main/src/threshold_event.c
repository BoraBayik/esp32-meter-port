#include "header/threshold_event.h"

#include <stddef.h>

// --- Zaman aritmetigi --------------------------------------------------------

// Howard Hinnant'in days_from_civil algoritmasi: 1970-01-01'den itibaren gun.
// Artik yillari dogru sayar, tablo gerektirmez.
static int32_t thDaysFromCivil(int32_t y, uint32_t m, uint32_t d) {
    y -= (m <= 2u);

    int32_t era = (y >= 0 ? y : y - 399) / 400;
    uint32_t yoe = (uint32_t)(y - era * 400);
    uint32_t doy = (153u * (m + (m > 2u ? (uint32_t)-3 : 9u)) + 2u) / 5u + d - 1u;
    uint32_t doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;

    return era * 146097 + (int32_t)doe - 719468;
}

uint32_t thTimeToMinutes(const th_time_t *t) {
    if (t == NULL) {
        return 0;
    }

    // RTC 2 haneli yil veriyor; 2000'li yillar varsayiliyor.
    int32_t year = 2000 + (int32_t)t->year;
    uint32_t month = (t->month >= 1 && t->month <= 12) ? (uint32_t)t->month : 1u;
    uint32_t day = (t->day >= 1 && t->day <= 31) ? (uint32_t)t->day : 1u;

    int32_t days = thDaysFromCivil(year, month, day);
    if (days < 0) {
        days = 0;
    }

    return (uint32_t)days * 1440u + (uint32_t)t->hour * 60u + (uint32_t)t->min;
}

uint32_t thMinutesBetween(const th_time_t *from, const th_time_t *to) {
    uint32_t a = thTimeToMinutes(from);
    uint32_t b = thTimeToMinutes(to);

    // Saat geri alinmis veya RTC bozulmussa negatif fark uretmeyelim.
    return (b > a) ? (b - a) : 0u;
}

// Iki zaman arasindaki sureyi kayit alanina sigacak sekilde dondurur.
// 0 dakikalik bir sure "veri yok" gibi gorunmesin diye en az 1 yapilir.
static uint16_t thDurationBetween(const th_time_t *start, const th_time_t *end) {
    uint32_t minutes = thMinutesBetween(start, end);

    if (minutes < 1u) {
        minutes = 1u;
    }
    if (minutes > TH_DURATION_MAX) {
        minutes = TH_DURATION_MAX;
    }

    return (uint16_t)minutes;
}

// --- Olay mantigi ------------------------------------------------------------

static void thClearTime(th_time_t *t) {
    t->year = 0;
    t->month = 0;
    t->day = 0;
    t->hour = 0;
    t->min = 0;
    t->sec = 0;
}

static void thEventClear(th_state_t *s) {
    s->active = false;
    s->just_resumed = false;
    s->above_count = 0;
    s->below_count = 0;
    s->peak_cv = 0;
}

void thEventInit(th_state_t *s, uint16_t enter_cv, uint16_t release_cv,
                 uint16_t enter_windows, uint16_t exit_windows,
                 uint32_t heartbeat_minutes) {
    if (s == NULL) {
        return;
    }

    s->enter_cv = enter_cv;
    s->release_cv = (release_cv < enter_cv) ? release_cv : enter_cv;
    s->enter_windows = (enter_windows == 0) ? 1u : enter_windows;
    s->exit_windows = (exit_windows == 0) ? 1u : exit_windows;
    s->heartbeat_minutes = heartbeat_minutes;

    thEventClear(s);
    thClearTime(&s->start_time);
    thClearTime(&s->drop_time);
    thClearTime(&s->last_record_time);
}

void thEventResume(th_state_t *s, const th_time_t *start_time, uint16_t peak_cv,
                   const th_time_t *last_record_time) {
    if (s == NULL || start_time == NULL || last_record_time == NULL) {
        return;
    }

    s->active = true;
    s->just_resumed = true;
    s->above_count = 0;
    s->below_count = 0;
    s->peak_cv = peak_cv;
    s->start_time = *start_time;
    s->last_record_time = *last_record_time;
    thClearTime(&s->drop_time);
}

th_action_t thEventWindow(th_state_t *s, uint16_t median_cv,
                          const th_time_t *now, th_record_t *out) {
    if (s == NULL || now == NULL || out == NULL) {
        return TH_ACTION_NONE;
    }

    if (!s->active) {
        if (median_cv < s->enter_cv) {
            // Esigin altina dusen tek pencere aday zinciri kirar.
            s->above_count = 0;
            return TH_ACTION_NONE;
        }

        // Olayin gercek baslangici, esigin ILK asildigi pencere.
        if (s->above_count == 0) {
            s->start_time = *now;
        }
        s->above_count++;

        if (s->above_count < s->enter_windows) {
            return TH_ACTION_NONE;
        }

        s->active = true;
        s->just_resumed = false;
        s->above_count = 0;
        s->below_count = 0;
        s->peak_cv = median_cv;
        s->last_record_time = s->start_time;

        out->time = s->start_time;
        out->vrms_cv = median_cv;
        out->duration = TH_DURATION_STARTED;
        return TH_ACTION_EVENT_STARTED;
    }

    // --- olay aktif ---
    if (median_cv > s->peak_cv) {
        s->peak_cv = median_cv;
    }

    if (median_cv < s->release_cv) {
        if (s->just_resumed) {
            // Acilistan sonraki ilk pencere zaten esigin altinda: ariza reset
            // sirasinda gecmis. Ne zaman gectigini bilmiyoruz; elimizdeki son
            // kanit flash'a en son yazilan kaydin zamani, olayi orada kapatiyoruz.
            out->time = s->last_record_time;
            out->vrms_cv = s->peak_cv;
            out->duration = thDurationBetween(&s->start_time, &s->last_record_time);

            thEventClear(s);
            return TH_ACTION_EVENT_ENDED;
        }

        // Esigin altina ILK dusus ani olayin gercek bitisidir; histerezis
        // gecikmesi sadece teyit icindir, kayda girmez.
        if (s->below_count == 0) {
            s->drop_time = *now;
        }
        s->below_count++;

        if (s->below_count >= s->exit_windows) {
            out->time = s->drop_time;
            out->vrms_cv = s->peak_cv;
            out->duration = thDurationBetween(&s->start_time, &s->drop_time);

            thEventClear(s);
            return TH_ACTION_EVENT_ENDED;
        }
    } else {
        // Esigin ustune geri cikti; cikis sayaci sifirlanir ki tek bir ariza
        // esik etrafinda salindigi icin onlarca olaya bolunmesin.
        s->below_count = 0;
        s->just_resumed = false;
    }

    if (s->heartbeat_minutes != 0 &&
        thMinutesBetween(&s->last_record_time, now) >= s->heartbeat_minutes) {
        s->last_record_time = *now;

        out->time = *now;
        out->vrms_cv = s->peak_cv;
        out->duration = TH_DURATION_ONGOING;
        return TH_ACTION_EVENT_ONGOING;
    }

    return TH_ACTION_NONE;
}

uint16_t thMedianCv(uint16_t *buf, uint16_t n) {
    if (buf == NULL || n == 0) {
        return 0;
    }

    // Pencere en fazla birkac on eleman; ekleme siralamasi fazlasiyla yeterli.
    for (uint16_t i = 1; i < n; i++) {
        uint16_t key = buf[i];
        uint16_t j = i;

        while (j > 0 && buf[j - 1] > key) {
            buf[j] = buf[j - 1];
            j--;
        }
        buf[j] = key;
    }

    if ((n & 1u) != 0u) {
        return buf[n / 2u];
    }

    return (uint16_t)(((uint32_t)buf[n / 2u - 1u] + (uint32_t)buf[n / 2u]) / 2u);
}

uint16_t thVoltsToCv(float volts) {
    if (!(volts > 0.0f)) { // NaN da buraya duser
        return 0;
    }

    float cv = volts * 100.0f;

    if (cv >= 65535.0f) {
        return 65535u;
    }

    return (uint16_t)(cv + 0.5f);
}

// --- Kayit cozumleme ---------------------------------------------------------

static bool thTwoDigits(const uint8_t *p, int *out) {
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') {
        return false;
    }

    *out = (p[0] - '0') * 10 + (p[1] - '0');
    return true;
}

bool thDecodeRecord(const uint8_t *raw, th_time_t *time, uint16_t *vrms_cv,
                    uint16_t *duration) {
    if (raw == NULL || time == NULL || vrms_cv == NULL || duration == NULL) {
        return false;
    }

    // Bos slot: silinmis (0xFF) veya eski kodun sifirladigi (0x00) alan.
    if (raw[0] == 0xFF || raw[0] == 0x00) {
        return false;
    }

    int fields[6];
    for (int i = 0; i < 6; i++) {
        if (!thTwoDigits(&raw[i * 2], &fields[i])) {
            return false; // ASCII tarih bozuk -> kaydi yok say
        }
    }

    time->year = (int16_t)fields[0];
    time->month = (int8_t)fields[1];
    time->day = (int8_t)fields[2];
    time->hour = (int8_t)fields[3];
    time->min = (int8_t)fields[4];
    time->sec = (int8_t)fields[5];

    if (time->month < 1 || time->month > 12 || time->day < 1 || time->day > 31 ||
        time->hour > 23 || time->min > 59 || time->sec > 59) {
        return false;
    }

    *vrms_cv = (uint16_t)(raw[12] | ((uint16_t)raw[13] << 8));
    *duration = (uint16_t)(raw[14] | ((uint16_t)raw[15] << 8));
    return true;
}

bool thFindOpenEvent(th_record_fetch_fn fetch, void *ctx, uint16_t max_back,
                     th_time_t *start_time, uint16_t *peak_cv,
                     th_time_t *last_record_time) {
    if (fetch == NULL || start_time == NULL || peak_cv == NULL ||
        last_record_time == NULL || max_back == 0) {
        return false;
    }

    uint8_t raw[16];
    th_time_t time;
    uint16_t vrms;
    uint16_t duration;

    // En son kayit acik degilse ortada devralinacak olay yok.
    if (!fetch(ctx, 1u, raw) || !thDecodeRecord(raw, &time, &vrms, &duration)) {
        return false;
    }
    if (!thDurationIsOpen(duration)) {
        return false;
    }

    *last_record_time = time;
    *peak_cv = vrms;
    *start_time = time; // BASLADI bulunamazsa geri dusulecek deger

    if (duration == TH_DURATION_STARTED) {
        return true; // zaten olayin ilk kaydi
    }

    // SURUYOR kaydi: gercek baslangic icin zincirde geriye yuru.
    for (uint16_t back = 2u; back <= max_back; back++) {
        if (!fetch(ctx, back, raw) ||
            !thDecodeRecord(raw, &time, &vrms, &duration)) {
            break; // bos slot veya bozuk kayit -> zincir burada bitiyor
        }

        if (duration == TH_DURATION_STARTED) {
            *start_time = time;
            break;
        }

        if (duration != TH_DURATION_ONGOING) {
            break; // kapali bir olaya denk geldik, zincir kirilmis
        }
    }

    return true;
}

// --- Halka tampon aritmetigi -------------------------------------------------

uint16_t thFindFreeOffset(const uint8_t *sector_data, uint16_t sector_size,
                          uint16_t record_size) {
    if (sector_data == NULL || record_size == 0) {
        return sector_size;
    }

    for (uint16_t offset = 0; offset + record_size <= sector_size;
         offset += record_size) {
        if (sector_data[offset] == 0xFF || sector_data[offset] == 0x00) {
            return offset;
        }
    }

    return sector_size;
}

th_write_pos_t thNextWritePos(uint16_t current_sector, uint16_t free_offset,
                              uint16_t sector_size, uint16_t record_size,
                              uint16_t sector_count) {
    th_write_pos_t pos;

    if (sector_count == 0 || record_size == 0) {
        pos.sector = current_sector;
        pos.slot_in_sector = 0;
        pos.sector_changed = false;
        return pos;
    }

    if (free_offset < sector_size) {
        pos.sector = (uint16_t)(current_sector % sector_count);
        pos.slot_in_sector = (uint16_t)(free_offset / record_size);
        pos.sector_changed = false;
        return pos;
    }

    // Sektor dolu: bir sonrakine gec. Son sektordeysek basa donulur ve o sektor
    // silinerek en eski kayitlar dusurulur.
    pos.sector = (uint16_t)((current_sector + 1u) % sector_count);
    pos.slot_in_sector = 0;
    pos.sector_changed = true;
    return pos;
}

uint16_t thSlotBack(uint16_t write_index, uint16_t back, uint16_t slot_count) {
    if (slot_count == 0) {
        return 0;
    }

    uint32_t idx = ((uint32_t)write_index % slot_count) + slot_count -
                   ((uint32_t)back % slot_count);

    return (uint16_t)(idx % slot_count);
}
