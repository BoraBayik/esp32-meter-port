#include "header/threshold_event.h"

#include <stddef.h>

void thEventInit(th_state_t *s, uint16_t enter_cv, uint16_t release_cv,
                 uint16_t enter_windows, uint16_t exit_windows,
                 uint32_t heartbeat_windows) {
    if (s == NULL) {
        return;
    }

    s->enter_cv = enter_cv;
    s->release_cv = (release_cv < enter_cv) ? release_cv : enter_cv;
    s->enter_windows = (enter_windows == 0) ? 1u : enter_windows;
    s->exit_windows = (exit_windows == 0) ? 1u : exit_windows;
    s->heartbeat_windows = heartbeat_windows;

    s->active = false;
    s->above_count = 0;
    s->below_count = 0;
    s->peak_cv = 0;
    s->minutes = 0;
    s->last_heartbeat = 0;

    s->candidate_time.year = 0;
    s->candidate_time.month = 0;
    s->candidate_time.day = 0;
    s->candidate_time.hour = 0;
    s->candidate_time.min = 0;
    s->candidate_time.sec = 0;
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
            s->candidate_time = *now;
        }
        s->above_count++;

        if (s->above_count < s->enter_windows) {
            return TH_ACTION_NONE;
        }

        s->active = true;
        s->below_count = 0;
        s->peak_cv = median_cv;
        s->minutes = s->above_count; // onay pencereleri de olaya dahildir
        s->last_heartbeat = 0;
        s->above_count = 0;

        out->time = s->candidate_time;
        out->vrms_cv = median_cv;
        out->duration = TH_DURATION_ONGOING;
        return TH_ACTION_EVENT_STARTED;
    }

    // --- olay aktif ---
    s->minutes++;

    if (median_cv > s->peak_cv) {
        s->peak_cv = median_cv;
    }

    if (median_cv < s->release_cv) {
        s->below_count++;

        if (s->below_count >= s->exit_windows) {
            // Esigin altinda gecen kuyruk pencereleri sureye dahil edilmez.
            uint32_t duration = (s->minutes > s->below_count)
                                    ? (s->minutes - s->below_count)
                                    : 1u;
            if (duration > TH_DURATION_MAX) {
                duration = TH_DURATION_MAX;
            }

            out->time = *now;
            out->vrms_cv = s->peak_cv;
            out->duration = (uint16_t)duration;

            s->active = false;
            s->above_count = 0;
            s->below_count = 0;
            s->minutes = 0;
            s->last_heartbeat = 0;
            return TH_ACTION_EVENT_ENDED;
        }
    } else {
        // Esigin ustune geri cikti; cikis sayaci sifirlanir ki tek bir ariza
        // esik etrafinda salindigi icin onlarca olaya bolunmesin.
        s->below_count = 0;
    }

    if (s->heartbeat_windows != 0 &&
        (s->minutes - s->last_heartbeat) >= s->heartbeat_windows) {
        s->last_heartbeat = s->minutes;

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
