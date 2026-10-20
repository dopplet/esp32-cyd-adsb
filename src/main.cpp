// ADS-B Radar Display â€” CYD (ESP32-2432S028)
// 320x240 landscape, TFT_eSPI direct rendering, no LVGL
// Features: radar, arrivals, stats, detail, flight log
//   compass rose, auto-cycle, brightness, night mode,
//   sort modes, filters, mil/emg alerts, closest approach record
#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <math.h>
#include <WiFi.h>

#include "config.h"
#include "data/aircraft.h"
#include "data/fetcher.h"
#include "data/storage.h"
#include "data/error_log.h"
#include "data/http_mutex.h"
#include "data/enrichment.h"
#include "ui/filters.h"

// Touch on separate VSPI bus
#define XPT2046_IRQ   36
#define XPT2046_MOSI  32
#define XPT2046_MISO  39
#define XPT2046_CLK   25
#define XPT2046_CS    33

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite status_spr(&tft);
SPIClass touchSPI = SPIClass(VSPI);

static AircraftList aircraft_list;

// ---- Color palette ----

static constexpr uint16_t c565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

struct ColorPalette {
    uint16_t bg;
    uint16_t grid;
    uint16_t ring;
    uint16_t sweep;
    uint16_t text;
    uint16_t trail;
    uint16_t blip;
    uint16_t blip_mil;
    uint16_t blip_emg;
    uint16_t status;
    uint16_t hdr;
    uint16_t status_bg;
    uint16_t fade_bright;
    uint16_t fade_med;
    uint16_t fade_dim;
    uint16_t fade_mil_dim;
    uint16_t fade_emg_dim;
};

static const ColorPalette PALETTE_GREEN = {
    TFT_BLACK,           // bg
    c565(0, 40, 0),      // grid
    c565(0, 60, 0),      // ring
    c565(0, 120, 0),     // sweep
    c565(0, 200, 0),     // text
    c565(0, 80, 0),      // trail
    TFT_GREEN,           // blip
    c565(255, 60, 60),   // blip_mil
    c565(255, 200, 0),   // blip_emg
    c565(0, 180, 0),     // status
    c565(0, 150, 0),     // hdr
    c565(0, 20, 0),      // status_bg
    c565(0, 255, 0),     // fade_bright
    c565(0, 140, 0),     // fade_med
    c565(0, 70, 0),      // fade_dim
    c565(120, 30, 30),   // fade_mil_dim
    c565(120, 90, 0),    // fade_emg_dim
};

static const ColorPalette PALETTE_NIGHT = {
    TFT_BLACK,           // bg
    c565(40, 10, 0),     // grid
    c565(60, 15, 0),     // ring
    c565(120, 40, 0),    // sweep
    c565(200, 80, 0),    // text
    c565(80, 30, 0),     // trail
    c565(255, 140, 0),   // blip (amber)
    c565(255, 60, 60),   // blip_mil
    c565(255, 50, 50),   // blip_emg (red)
    c565(180, 70, 0),    // status
    c565(150, 60, 0),    // hdr
    c565(20, 5, 0),      // status_bg
    c565(255, 140, 0),   // fade_bright
    c565(140, 60, 0),    // fade_med
    c565(70, 25, 0),     // fade_dim
    c565(120, 30, 30),   // fade_mil_dim
    c565(120, 20, 20),   // fade_emg_dim
};

static const ColorPalette *pal = &PALETTE_GREEN;

// ---- Geometry ----

#define STATUS_H  20
#define RADAR_Y   STATUS_H
#define RADAR_H   (LCD_V_RES - STATUS_H)
#define RADAR_CX  (LCD_H_RES / 2)
#define RADAR_CY  (RADAR_Y + RADAR_H / 2)
#define RADAR_R   (RADAR_H / 2 - 4)

// Range options (nm)
static const float RANGES[] = {ADSB_RADIUS_NM, 10, 5, 1};
static const int NUM_RANGES = 4;
static int range_idx = 0;

// ---- View state ----

enum ViewMode { VIEW_LOADING, VIEW_RADAR, VIEW_INFO, VIEW_SETTINGS };
static ViewMode current_view = VIEW_LOADING;
static bool view_changed = true;

// Sweep
static float sweep_angle = 0;
static float prev_sweep_angle = -1;
#define SWEEP_SPEED 3.0f

// ---- Differential rendering ----

struct BlipState {
    int16_t x, y;
    int16_t hx, hy;
    uint8_t r;
    bool has_heading;
    bool has_label;
    bool has_info;
};
static BlipState prev_blips[MAX_AIRCRAFT];
static int prev_blip_count = 0;
static bool radar_needs_full_redraw = true;

struct TrailState {
    int16_t x[TRAIL_LENGTH];
    int16_t y[TRAIL_LENGTH];
    uint8_t count;
    uint8_t style;  // 0=line, 1=dots — captured at draw time so erase matches
};
static TrailState prev_trails[MAX_AIRCRAFT];
static int prev_trail_count = 0;

// ---- Auto-cycle ----

static uint32_t last_touch_time = 0;
static uint32_t last_cycle_time = 0;


// ---- Alert state ----

struct AlertState {
    bool active;
    uint32_t start_time;
    char message[24];
    uint16_t color;
};
static AlertState alert = {};
#define ALERT_DURATION_MS 5000
#define ALERT_DEDUP_SIZE 8
static uint32_t alerted_icaos[ALERT_DEDUP_SIZE];
static int alerted_idx = 0;

// ---- Closest approach record ----

struct ClosestRecord {
    char callsign[9];
    char type_code[5];
    float distance_nm;
    int32_t altitude;
    uint32_t timestamp;
};
static ClosestRecord closest_record = {"---", "---", 9999.0f, 0, 0};

// ---- Flight log ----

struct LogEntry {
    uint32_t icao_hash;
    char callsign[9];
    char type_code[5];
    uint32_t first_seen;
    float closest_nm;
    bool is_military;
    bool is_emergency;
};
#define LOG_SIZE 50
static LogEntry flight_log[LOG_SIZE];
static int log_count = 0;
static int log_write_idx = 0;
static int log_page = 0;

// ---- Info view state ----

static char g_info_icao[7] = "";
static AircraftEnrichment *g_info_enrich = nullptr;
static uint32_t last_info_draw = 0;

// ---- Long-press detection ----

static uint32_t touch_down_time = 0;
static bool long_press_fired = false;
static int saved_tx = 0;
static int saved_ty = 0;

// ---- Settings view ----
#define NUM_SETTINGS 4
static int settings_sel = -1;  // -1 = none, 0..N-1 = selected row (options shown)
static const char *SETTINGS_LABELS[] = {
    "ALERT MILITARY", "ALERT EMERGENCY", "TRAILS", "TRAIL STYLE"
};
#define LONG_PRESS_MS 1000
#define SETTINGS_ROW_H   22
#define SETTINGS_FIRST_Y (RADAR_Y + 23)
#define SETTINGS_OPT_Y   (SETTINGS_FIRST_Y + NUM_SETTINGS * SETTINGS_ROW_H + 6)

// ---- Helpers ----

static float distance_nm(float lat, float lon) {
    float dlat = lat - HOME_LAT;
    float dlon = lon - HOME_LON;
    float cos_lat = cosf(HOME_LAT * M_PI / 180.0f);
    float nm_north = dlat * 60.0f;
    float nm_east = dlon * 60.0f * cos_lat;
    return sqrtf(nm_north * nm_north + nm_east * nm_east);
}

static bool find_nearest_aircraft(char *out_icao, float *out_dist) {
    float best = 2.0f;  // only show aircraft within 2nm
    bool found = false;
    uint32_t now = millis();
    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            Aircraft &a = aircraft_list.aircraft[i];
            if (a.lat == 0.0f && a.lon == 0.0f) continue;
            if (compute_aircraft_opacity(a.stale_since, now) == 0) continue;
            float d = distance_nm(a.lat, a.lon);
            if (d < best) {
                best = d;
                strlcpy(out_icao, a.icao_hex, 7);
                if (out_dist) *out_dist = d;
                found = true;
            }
        }
        aircraft_list.unlock();
    }
    return found;
}

static void info_enrichment_cb(AircraftEnrichment *data) {
    g_info_enrich = data;
    view_changed = true;
}

// Called back from fetcher task when /v2/closest/ resolves
static void closest_result_cb(const char *icao_hex) {
    if (!icao_hex || !icao_hex[0]) return;
    strlcpy(g_info_icao, icao_hex, sizeof(g_info_icao));
    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            Aircraft &a = aircraft_list.aircraft[i];
            if (strcmp(a.icao_hex, icao_hex) == 0) {
                enrichment_fetch(icao_hex, a.registration, a.callsign, info_enrichment_cb);
                break;
            }
        }
        aircraft_list.unlock();
    }
    view_changed = true;
}

static void enter_info_view() {
    g_info_enrich = nullptr;
    g_info_icao[0] = '\0';
    fetcher_request_closest(HOME_LAT, HOME_LON, ADSB_RADIUS_NM, closest_result_cb);
    current_view = VIEW_INFO;
    view_changed = true;
    last_info_draw = 0;
    tft.fillRect(0, STATUS_H, LCD_H_RES, RADAR_H, pal->bg);
}

static bool latlon_to_radar(float lat, float lon, int &px, int &py) {
    float range_nm = RANGES[range_idx];
    float dlat = lat - HOME_LAT;
    float dlon = lon - HOME_LON;
    float cos_lat = cosf(HOME_LAT * M_PI / 180.0f);
    float nm_north = dlat * 60.0f;
    float nm_east = dlon * 60.0f * cos_lat;
    if (sqrtf(nm_north * nm_north + nm_east * nm_east) > range_nm) return false;
    float scale = RADAR_R / range_nm;
    px = RADAR_CX + (int)(nm_east * scale);
    py = RADAR_CY - (int)(nm_north * scale);
    return true;
}

static uint32_t hash_icao(const char *hex) {
    uint32_t h = 0;
    for (int i = 0; hex[i]; i++) h = h * 31 + hex[i];
    return h;
}

// ---- Touch ----

static uint16_t touchRead16(uint8_t cmd) {
    digitalWrite(XPT2046_CS, LOW);
    touchSPI.beginTransaction(SPISettings(2500000, MSBFIRST, SPI_MODE0));
    touchSPI.transfer(cmd);
    uint16_t val = touchSPI.transfer16(0);
    touchSPI.endTransaction();
    digitalWrite(XPT2046_CS, HIGH);
    return val >> 3;
}

static bool getTouchPoint(int &tx, int &ty) {
    if (digitalRead(XPT2046_IRQ) != LOW) return false;
    uint32_t sumX = 0, sumY = 0;
    for (int i = 0; i < 4; i++) {
        sumX += touchRead16(0x90);  // Y+ electrode = horizontal axis on CYD landscape
        sumY += touchRead16(0xD0);  // X+ electrode = vertical axis on CYD landscape
    }
    tx = map(sumX / 4, TOUCH_X_MIN, TOUCH_X_MAX, 0, 319);
    ty = map(sumY / 4, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, 239);
    tx = constrain(tx, 0, 319);
    ty = constrain(ty, 0, 239);
    return true;
}

// ---- Brightness ----

static void apply_brightness() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(TFT_BL, g_config.brightness);
#else
    ledcWrite(0, g_config.brightness);
#endif
}

static void adjust_brightness(int delta) {
    int b = (int)g_config.brightness + delta;
    if (b < 32) b = 32;
    if (b > 255) b = 255;
    g_config.brightness = (uint8_t)b;
    apply_brightness();
    storage_save_config(g_config);
}

// ---- Night mode ----

static void toggle_night_mode() {
    g_config.night_mode = !g_config.night_mode;
    pal = g_config.night_mode ? &PALETTE_NIGHT : &PALETTE_GREEN;
    storage_save_config(g_config);
    // Force full redraw of everything
    tft.fillScreen(pal->bg);
    radar_needs_full_redraw = true;
    view_changed = true;
}

// ---- View cycling ----

static void cycle_view() {
    if (current_view == VIEW_SETTINGS) {
        current_view = VIEW_RADAR;
        radar_needs_full_redraw = true;
        settings_sel = -1;
    }
    view_changed = true;
}

static void handle_touch_info(int tx, int ty) {
    // Bottom-right back button (radar glyph)
    if (tx > 270 && ty > 205) {
        current_view = VIEW_RADAR;
        radar_needs_full_redraw = true;
        view_changed = true;
        tft.fillRect(0, STATUS_H, LCD_H_RES, RADAR_H, pal->bg);
        return;
    }
    // Header bar: same zones as radar view
    if (ty < STATUS_H) {
        if (tx < 40) {
            current_view = VIEW_SETTINGS;
            view_changed = true;
            tft.fillRect(0, STATUS_H, LCD_H_RES, RADAR_H, pal->bg);
        } else if (tx <= 220) {
            filter_cycle();
        } else {
            range_idx = (range_idx + 1) % NUM_RANGES;
        }
    }
}

// ---- Touch handlers per view ----

static void handle_touch_radar(int tx, int ty) {
    // Info view button (bottom-right tile)
    if (tx > 270 && ty > 205) {
        enter_info_view();
        return;
    }
    if (ty >= STATUS_H) return;
    if (tx < 40) {
        // WiFi label â†’ go to settings
        current_view = VIEW_SETTINGS;
        view_changed = true;
    } else if (tx <= 220) {
        // AC count / filter label â†’ cycle filter
        filter_cycle();
        radar_needs_full_redraw = true;
    } else {
        // Range label â†’ cycle range
        range_idx = (range_idx + 1) % NUM_RANGES;
        radar_needs_full_redraw = true;
    }
}

static int get_setting_options(int sel, const char **labels, int *active_opt) {
    switch (sel) {
        case 0:
            labels[0] = "OFF"; labels[1] = "ON";
            *active_opt = g_config.alert_military ? 1 : 0;
            return 2;
        case 1:
            labels[0] = "OFF"; labels[1] = "ON";
            *active_opt = g_config.alert_emergency ? 1 : 0;
            return 2;
        case 2:
            labels[0] = "OFF"; labels[1] = "ON";
            *active_opt = g_config.trails_enabled ? 1 : 0;
            return 2;
        case 3:
            labels[0] = "LINE"; labels[1] = "DOTS";
            *active_opt = g_config.trail_style;
            return 2;
        default: return 0;
    }
}

static void apply_setting(int sel, int opt) {
    switch (sel) {
        case 0: g_config.alert_military  = (opt == 1); break;
        case 1: g_config.alert_emergency = (opt == 1); break;
        case 2: g_config.trails_enabled  = (opt == 1); break;
        case 3: g_config.trail_style     = opt;         break;
    }
    storage_save_config(g_config);
}

static void handle_touch_settings(int tx, int ty) {
    if (ty < STATUS_H) {
        if (tx < 40) cycle_view();
        return;
    }
    if (ty < SETTINGS_FIRST_Y) {
        cycle_view();
        return;
    }
    if (settings_sel >= 0 && ty >= SETTINGS_OPT_Y) {
        const char *labels[3];
        int active_opt;
        int n_opts = get_setting_options(settings_sel, labels, &active_opt);
        if (n_opts > 0) {
            int opt = constrain(tx * n_opts / LCD_H_RES, 0, n_opts - 1);
            apply_setting(settings_sel, opt);
        }
        settings_sel = -1;
        view_changed = true;
        return;
    }
    if (ty < SETTINGS_FIRST_Y + NUM_SETTINGS * SETTINGS_ROW_H) {
        int row = constrain((ty - SETTINGS_FIRST_Y) / SETTINGS_ROW_H, 0, NUM_SETTINGS - 1);
        settings_sel = (settings_sel == row) ? -1 : row;
        view_changed = true;
    }
}

// ---- Status bar ----

static uint32_t last_status_draw = 0;

static void draw_status_bar(bool force) {
    uint32_t now = millis();
    if (!force && now - last_status_draw < 1000) return;
    last_status_draw = now;

    // Draw into sprite â€” pushed to display in one SPI transfer, no visible blank frame
    status_spr.fillSprite(pal->status_bg);
    status_spr.setTextColor(pal->status, pal->status_bg);
    status_spr.setTextDatum(TL_DATUM);

    // WiFi status
    const char *net = fetcher_wifi_connected() ? "WiFi" : "---";
    status_spr.drawString(net, 2, 3, 2);

    // Aircraft count
    char buf[24];
    const char *vn = current_view == VIEW_SETTINGS ? "SET" :
                     current_view == VIEW_INFO     ? "NFO" : "RDR";
    snprintf(buf, sizeof(buf), "%s %dAC", vn, aircraft_list.count);
    status_spr.drawString(buf, 45, 3, 2);

    // Filter indicator (if active)
    int filt = filter_get_active();
    if (filt >= 0 && filt < NUM_FILTERS) {
        status_spr.setTextColor(filter_defs[filt].color, pal->status_bg);
        status_spr.drawString(filter_defs[filt].label, 120, 3, 2);
        status_spr.setTextColor(pal->status, pal->status_bg);
    }

    // Range
    status_spr.setTextDatum(TR_DATUM);
    snprintf(buf, sizeof(buf), "%dnm", (int)RANGES[range_idx]);
    status_spr.drawString(buf, 316, 3, 2);

    // Last update age
    uint32_t age = (now - fetcher_last_update()) / 1000;
    if (fetcher_last_update() > 0) {
        snprintf(buf, sizeof(buf), "%lus", age);
        status_spr.drawString(buf, 260, 3, 2);
    }

    // Free heap (tiny font center)
    snprintf(buf, sizeof(buf), "%luK", (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    status_spr.setTextDatum(MC_DATUM);
    status_spr.drawString(buf, 160, 14, 1);

    status_spr.pushSprite(0, 0);
}

// ---- Loading animation ----

static float load_sweep = 0;
static int load_dots = 0;
static uint32_t last_dot_time = 0;

static void draw_loading() {
    const int cx = LCD_H_RES / 2;
    const int cy = LCD_V_RES / 2 - 10;
    const int r = 60;

    // Erase previous sweep line
    float prev = load_sweep - 4.0f;
    if (prev < 0) prev += 360.0f;
    float prad = prev * M_PI / 180.0f;
    tft.drawLine(cx, cy, cx + (int)(r * sinf(prad)), cy - (int)(r * cosf(prad)), pal->bg);

    // Rings + crosshair
    tft.drawCircle(cx, cy, r, pal->ring);
    tft.drawCircle(cx, cy, r * 2 / 3, pal->ring);
    tft.drawCircle(cx, cy, r / 3, pal->ring);
    tft.drawLine(cx - r, cy, cx + r, cy, pal->grid);
    tft.drawLine(cx, cy - r, cx, cy + r, pal->grid);

    // Sweep
    float srad = load_sweep * M_PI / 180.0f;
    tft.drawLine(cx, cy, cx + (int)(r * sinf(srad)), cy - (int)(r * cosf(srad)), pal->sweep);
    tft.fillCircle(cx, cy, 2, pal->text);

    load_sweep += 4.0f;
    if (load_sweep >= 360.0f) load_sweep -= 360.0f;

    // Status text
    int ty = cy + r + 16;
    tft.setTextDatum(MC_DATUM);
    uint32_t now = millis();

    if (now - last_dot_time > 500) { last_dot_time = now; load_dots = (load_dots + 1) % 4; }
    const char *dots = load_dots == 1 ? "." : load_dots == 2 ? ".." : load_dots == 3 ? "..." : "   ";

    if (!fetcher_wifi_connected()) {
        char msg[20];
        snprintf(msg, sizeof(msg), "WiFi%-3s", dots);
        tft.setTextColor(pal->text, pal->bg);
        tft.drawString(msg, cx, ty, 2);
    } else if (fetcher_last_update() == 0) {
        char msg[24];
        snprintf(msg, sizeof(msg), "Scanning%-3s", dots);
        tft.setTextColor(pal->sweep, pal->bg);
        tft.drawString(msg, cx, ty, 2);
        // Show IP once connected
        const FetcherStats *fs = fetcher_get_stats();
        if (fs->ip_addr[0]) {
            tft.setTextColor(pal->fade_med, pal->bg);
            tft.drawString(fs->ip_addr, cx, ty + 18, 1);
        }
    } else {
        // Brief "found N aircraft" before transitioning
        char msg[32];
        snprintf(msg, sizeof(msg), "Found %d aircraft", aircraft_list.count);
        tft.setTextColor(pal->sweep, pal->bg);
        tft.drawString(msg, cx, ty, 2);
    }

    tft.setTextColor(pal->text, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("ADS-B Radar", cx, cy - r - 20, 4);
}

// ---- Radar view ----

static void draw_radar_glyph(int cx, int cy, uint16_t col) {
    tft.drawCircle(cx, cy, 10, col);              // outer ring
    tft.drawCircle(cx, cy,  6, col);              // mid ring
    tft.drawCircle(cx, cy,  2, col);              // inner ring
    tft.drawLine(cx-10, cy, cx+10, cy, col);      // horizontal crosshair
    tft.drawLine(cx, cy-10, cx, cy+10, col);      // vertical crosshair
    tft.fillCircle(cx, cy, 1, col);               // centre dot
}

static const uint8_t AIRPLANE_BMP[] PROGMEM = {
    0x01, 0x80,  // row  0: ......xx........
    0x03, 0xC0,  // row  1: .....xxxx.......
    0x03, 0xC0,  // row  2: .....xxxx.......
    0x03, 0xC0,  // row  3: .....xxxx.......
    0x03, 0xC0,  // row  4: .....xxxx.......
    0x07, 0xE0,  // row  5: ....xxxxxx......
    0x0F, 0xF0,  // row  6: ...xxxxxxxx.....
    0x1F, 0xF8,  // row  7: ..xxxxxxxxxx....
    0x3F, 0xFC,  // row  8: .xxxxxxxxxxxx...
    0x7B, 0xDE,  // row  9: .xxxx.xxxx.xxxx.
    0xE3, 0xC7,  // row 10: xxx....xxxx....x  (note: MSB=left)
    0x83, 0xC1,  // row 11: x......xxxx.....  (note: MSB=left)
    0x03, 0xC0,  // row 12: .....xxxx.......
    0x0F, 0xF0,  // row 13: ...xxxxxxxx.....
    0x1F, 0xF8,  // row 14: ..xxxxxxxxxx....
    0x39, 0x9C,  // row 15: ..xxx..xx..xxx..
};

static void draw_plane_glyph(int cx, int cy, uint16_t col) {
    tft.drawBitmap(cx - 8, cy - 8, AIRPLANE_BMP, 16, 16, col);
}

static void draw_radar() {
    if (radar_needs_full_redraw) {
        tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
        radar_needs_full_redraw = false;
        prev_blip_count = 0;
        prev_trail_count = 0;
        prev_sweep_angle = -1;
        // Draw rings immediately so they're visible even when no aircraft match the filter.
        // The incremental repair block is gated on aircraft presence + new data, so without
        // this initial paint the rings never appear on empty-filter screens.
        for (int i = 1; i <= 3; i++) {
            int r = RADAR_R * i / 3;
            tft.drawCircle(RADAR_CX, RADAR_CY, r, pal->ring);
        }
        tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, pal->grid);
        tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, pal->grid);
        tft.setTextColor(pal->ring, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("N", RADAR_CX, RADAR_CY - RADAR_R + 6, 1);
        tft.drawString("S", RADAR_CX, RADAR_CY + RADAR_R - 6, 1);
        tft.drawString("E", RADAR_CX + RADAR_R - 6, RADAR_CY, 1);
        tft.drawString("W", RADAR_CX - RADAR_R + 6, RADAR_CY, 1);
    }

    // Erase old sweep
    if (prev_sweep_angle >= 0) {
        float prad = prev_sweep_angle * M_PI / 180.0f;
        tft.drawLine(RADAR_CX, RADAR_CY,
            RADAR_CX + (int)(RADAR_R * sinf(prad)),
            RADAR_CY - (int)(RADAR_R * cosf(prad)), pal->bg);
    }

    // Erase old trails
    for (int i = 0; i < prev_trail_count; i++) {
        TrailState &t = prev_trails[i];
        if (t.style == 1) {
            for (int j = 0; j < t.count; j++)
                tft.drawPixel(t.x[j], t.y[j], pal->bg);
        } else {
            for (int j = 1; j < t.count; j++)
                tft.drawLine(t.x[j-1], t.y[j-1], t.x[j], t.y[j], pal->bg);
        }
    }

    // Erase old blips
    for (int i = 0; i < prev_blip_count; i++) {
        BlipState &b = prev_blips[i];
        tft.fillCircle(b.x, b.y, b.r, pal->bg);
        if (b.has_heading) tft.drawLine(b.x, b.y, b.hx, b.hy, pal->bg);
        if (b.has_label) tft.fillRect(b.x + 4, b.y - 12, 76, b.has_info ? 24 : 12, pal->bg);
    }

    // Repair rings every frame: the sweep erases one ring-crossing per frame
    // unconditionally, so a data-gated repair (every ~7.5s) leaves up to 113
    // damaged pixels accumulated across the ring circumference — essentially
    // the full ring eaten away before each repair. Cost: ~1326 drawPixel calls
    // per frame at 15fps, negligible on ESP32/ILI9341.
    for (int i = 1; i <= 3; i++) {
        int r = RADAR_R * i / 3;
        tft.drawCircle(RADAR_CX, RADAR_CY, r, pal->ring);
    }
    tft.drawLine(RADAR_CX - RADAR_R, RADAR_CY, RADAR_CX + RADAR_R, RADAR_CY, pal->grid);
    tft.drawLine(RADAR_CX, RADAR_CY - RADAR_R, RADAR_CX, RADAR_CY + RADAR_R, pal->grid);

    // Sweep line
    float rad = sweep_angle * M_PI / 180.0f;
    int sx = RADAR_CX + (int)(RADAR_R * sinf(rad));
    int sy = RADAR_CY - (int)(RADAR_R * cosf(rad));
    tft.drawLine(RADAR_CX, RADAR_CY, sx, sy, pal->sweep);
    prev_sweep_angle = sweep_angle;

    // Home dot
    tft.fillCircle(RADAR_CX, RADAR_CY, 2, pal->text);

    // Aircraft
    int new_blip_count = 0;
    int new_trail_count = 0;
    if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count && new_blip_count < MAX_AIRCRAFT; i++) {
            Aircraft &a = aircraft_list.aircraft[i];

            // Apply filter
            if (!aircraft_passes_filter(a)) continue;

            int px, py;
            if (!latlon_to_radar(a.lat, a.lon, px, py)) continue;

            int dx = px - RADAR_CX;
            int dy = py - RADAR_CY;
            if (dx * dx + dy * dy > RADAR_R * RADAR_R) continue;

            uint8_t opacity = compute_aircraft_opacity(a.stale_since, millis());
            if (opacity == 0) continue;

            // Trail
            if (g_config.trails_enabled && a.trail_count > 1 && new_trail_count < MAX_AIRCRAFT) {
                TrailState &ts = prev_trails[new_trail_count];
                ts.count = 0;
                ts.style = g_config.trail_style;
                for (int t = 0; t < a.trail_count && ts.count < TRAIL_LENGTH; t++) {
                    int tx, ty;
                    if (latlon_to_radar(a.trail[t].lat, a.trail[t].lon, tx, ty)) {
                        ts.x[ts.count] = tx;
                        ts.y[ts.count] = ty;
                        ts.count++;
                    }
                }
                if (ts.style == 1) {
                    for (int t = 0; t < ts.count; t++)
                        tft.drawPixel(ts.x[t], ts.y[t], pal->trail);
                } else {
                    for (int t = 1; t < ts.count; t++)
                        tft.drawLine(ts.x[t-1], ts.y[t-1], ts.x[t], ts.y[t], pal->trail);
                }
                new_trail_count++;
            }

            // Sweep-angle fade
            float ac_angle = atan2f((float)(px - RADAR_CX), (float)(RADAR_CY - py)) * 180.0f / M_PI;
            if (ac_angle < 0) ac_angle += 360.0f;
            float behind = sweep_angle - ac_angle;
            if (behind < 0) behind += 360.0f;

            uint16_t fade_color;
            if (a.is_emergency)
                fade_color = (behind < 60) ? pal->blip_emg : pal->fade_emg_dim;
            else if (a.is_military)
                fade_color = (behind < 60) ? pal->blip_mil : pal->fade_mil_dim;
            else if (behind < 60)
                fade_color = pal->fade_bright;
            else if (behind < 180)
                fade_color = pal->fade_med;
            else
                fade_color = pal->fade_dim;

            int blip_r = (a.is_military || a.is_emergency) ? 3 : 2;
            tft.fillCircle(px, py, blip_r, fade_color);

            BlipState &b = prev_blips[new_blip_count];
            b.x = px; b.y = py; b.r = blip_r;
            b.has_heading = false; b.has_label = false; b.has_info = false;

            // Heading line
            if (a.heading > 0 && !a.on_ground) {
                float hrad = a.heading * M_PI / 180.0f;
                int hx = px + (int)(8 * sinf(hrad));
                int hy = py - (int)(8 * cosf(hrad));
                tft.drawLine(px, py, hx, hy, fade_color);
                b.has_heading = true; b.hx = hx; b.hy = hy;
            }

            // Callsign always visible; alt/speed only when freshly swept
            tft.setTextColor(fade_color, pal->bg);
            if (a.callsign[0]) {
                tft.setTextDatum(BL_DATUM);
                tft.drawString(a.callsign, px + 4, py - 2, 1);
                b.has_label = true;
            }
            if (behind < 60) {
                char info[20];
                if (a.altitude > 0)
                    snprintf(info, sizeof(info), "%d %dk", (int)a.altitude, a.speed);
                else if (a.on_ground)
                    snprintf(info, sizeof(info), "GND %dk", a.speed);
                else
                    info[0] = '\0';
                if (info[0]) {
                    tft.setTextDatum(TL_DATUM);
                    tft.drawString(info, px + 4, py + 2, 1);
                    b.has_info = true;
                }
            }

            new_blip_count++;
        }
        aircraft_list.unlock();
    }
    prev_blip_count = new_blip_count;
    prev_trail_count = new_trail_count;

    // Range labels (drawn last to stay readable over aircraft)
    tft.setTextColor(pal->ring, pal->bg);
    tft.setTextDatum(TC_DATUM);
    for (int i = 1; i <= 3; i++) {
        int r = RADAR_R * i / 3;
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", (int)(RANGES[range_idx] * i / 3));
        tft.drawString(buf, RADAR_CX + 2, RADAR_CY - r + 2, 1);
    }

    // Info view button — bottom-right, outside radar circle
    tft.fillRect(291, 219, 28, 20, pal->grid);
    draw_plane_glyph(305, 229, pal->text);
}

// ---- Settings screen ----

static void draw_settings() {
    if (!view_changed) return;
    tft.fillRect(0, RADAR_Y, LCD_H_RES, RADAR_H, pal->bg);
    view_changed = false;

    int y = RADAR_Y + 2;
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("SETTINGS", 4, y, 2);

    y += 17;
    tft.drawLine(0, y, LCD_H_RES, y, pal->hdr);
    y += 4;  // y = SETTINGS_FIRST_Y

    char buf[16];
    for (int i = 0; i < NUM_SETTINGS; i++) {
        bool sel = (i == settings_sel);
        uint16_t row_bg = sel ? pal->grid : pal->bg;
        tft.fillRect(0, y, LCD_H_RES, SETTINGS_ROW_H, row_bg);

        tft.setTextColor(sel ? pal->blip : pal->hdr, row_bg);
        tft.setTextDatum(TL_DATUM);
        tft.drawString(SETTINGS_LABELS[i], 8, y + 3, 2);

        switch (i) {
            case 0: strlcpy(buf, g_config.alert_military  ? "ON" : "OFF", sizeof(buf)); break;
            case 1: strlcpy(buf, g_config.alert_emergency ? "ON" : "OFF", sizeof(buf)); break;
            case 2: strlcpy(buf, g_config.trails_enabled  ? "ON" : "OFF", sizeof(buf)); break;
            case 3: strlcpy(buf, g_config.trail_style     ? "DOTS" : "LINE", sizeof(buf)); break;
        }
        tft.setTextColor(sel ? pal->blip : pal->text, row_bg);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(buf, 308, y + 3, 2);

        y += SETTINGS_ROW_H;
    }
    // y = SETTINGS_FIRST_Y + NUM_SETTINGS * SETTINGS_ROW_H = 153

    y += 2;
    tft.drawLine(0, y, LCD_H_RES, y, pal->grid);
    y += 4;  // y = SETTINGS_OPT_Y = 159

    if (settings_sel >= 0) {
        const char *labels[3];
        int active_opt;
        int n_opts = get_setting_options(settings_sel, labels, &active_opt);
        int btn_w = LCD_H_RES / n_opts;
        const int btn_h = 40;

        for (int i = 0; i < n_opts; i++) {
            bool active = (i == active_opt);
            uint16_t btn_bg = active ? pal->blip : pal->grid;
            uint16_t btn_fg = active ? pal->bg : pal->text;
            tft.fillRect(i * btn_w + 2, y, btn_w - 4, btn_h, btn_bg);
            tft.setTextColor(btn_fg, btn_bg);
            tft.setTextDatum(MC_DATUM);
            tft.drawString(labels[i], i * btn_w + btn_w / 2, y + btn_h / 2, 2);
        }
    } else {
        tft.setTextColor(pal->grid, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("tap row to edit", LCD_H_RES / 2, y + 20, 1);
    }
}

// ---- Info view ----

static void draw_info() {
    uint32_t now = millis();
    if (!view_changed && now - last_info_draw < 1000) return;
    view_changed = false;
    last_info_draw = now;

    tft.fillRect(0, STATUS_H, LCD_H_RES, RADAR_H, pal->bg);

    // Snapshot nearest aircraft from live list
    Aircraft snap;
    bool found = false;
    float dist_val = 0.0f;

    if (g_info_icao[0] && aircraft_list.lock(pdMS_TO_TICKS(50))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            if (strcmp(aircraft_list.aircraft[i].icao_hex, g_info_icao) == 0) {
                snap = aircraft_list.aircraft[i];
                dist_val = distance_nm(snap.lat, snap.lon);
                found = true;
                break;
            }
        }
        aircraft_list.unlock();
    }

    // If locked aircraft left range, reacquire nearest
    if (!found) {
        char alt_icao[7] = "";
        if (find_nearest_aircraft(alt_icao, &dist_val)) {
            strlcpy(g_info_icao, alt_icao, sizeof(g_info_icao));
            enrichment_fetch(g_info_icao, "", "", info_enrichment_cb);
            if (aircraft_list.lock(pdMS_TO_TICKS(50))) {
                for (int i = 0; i < aircraft_list.count; i++) {
                    if (strcmp(aircraft_list.aircraft[i].icao_hex, g_info_icao) == 0) {
                        snap = aircraft_list.aircraft[i];
                        found = true;
                        break;
                    }
                }
                aircraft_list.unlock();
            }
        }
    }

    if (!found) {
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.setTextDatum(MC_DATUM);
        int mid_y = STATUS_H + RADAR_H / 2;
        tft.drawString("NO AIRCRAFT", LCD_H_RES / 2, mid_y - 14, 4);
        tft.drawString("OVERHEAD",    LCD_H_RES / 2, mid_y + 14, 4);
        tft.fillRect(291, 215, 28, 24, pal->grid);
        draw_radar_glyph(305, 227, pal->text);
        return;
    }

    AircraftEnrichment *enr = g_info_enrich ? g_info_enrich : enrichment_get_cached(g_info_icao);

    char buf[64];

    // ── Identity ─────────────────────────────────────────────────────────

    tft.setTextColor(pal->text, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(snap.callsign[0] ? snap.callsign : snap.icao_hex, 4, 22, 3);

    // Badges (MIL/EMG) and airline — top-right
    int badge_y = 22;
    if (snap.is_military) {
        tft.setTextColor(pal->blip_mil, pal->bg);
        tft.setTextDatum(TR_DATUM);
        tft.drawString("MIL", 316, badge_y, 1);
        badge_y += 10;
    }
    if (snap.is_emergency) {
        tft.setTextColor(pal->blip_emg, pal->bg);
        tft.setTextDatum(TR_DATUM);
        tft.drawString("EMG", 316, badge_y, 1);
        badge_y += 10;
    }
    const char *airline_s = (enr && enr->airline[0]) ? enr->airline : snap.owner_op;
    if (airline_s[0]) {
        tft.setTextColor(pal->hdr, pal->bg);
        tft.setTextDatum(TR_DATUM);
        tft.drawString(airline_s, 316, badge_y, 1);
    }

    // Registration · type code · description
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    const char *desc_s = snap.desc[0] ? snap.desc : (enr && enr->model[0] ? enr->model : "");
    char desc_trunc[30] = "";
    if (desc_s[0]) strlcpy(desc_trunc, desc_s, sizeof(desc_trunc));
    if (desc_trunc[0])
        snprintf(buf, sizeof(buf), "%s  %s  %s",
            snap.registration[0] ? snap.registration : "---",
            snap.type_code[0] ? snap.type_code : "---",
            desc_trunc);
    else
        snprintf(buf, sizeof(buf), "%s  %s",
            snap.registration[0] ? snap.registration : "---",
            snap.type_code[0] ? snap.type_code : "---");
    tft.drawString(buf, 4, 47, 1);

    // ICAO hex + category (dim)
    tft.setTextColor(pal->fade_dim, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(snap.icao_hex, 4, 58, 1);
    if (snap.category[0]) {
        tft.setTextDatum(TR_DATUM);
        tft.drawString(snap.category, 316, 58, 1);
    }

    tft.drawLine(0, 68, LCD_H_RES, 68, pal->grid);

    // ── Live Stats ───────────────────────────────────────────────────────

    const int RX = 162;  // right column X start

    // ALT
    tft.setTextDatum(TL_DATUM);
    if (snap.on_ground)
        strlcpy(buf, "GND", sizeof(buf));
    else if (snap.altitude > 0)
        snprintf(buf, sizeof(buf), "%ld ft", (long)snap.altitude);
    else
        strlcpy(buf, "---", sizeof(buf));
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("ALT", 4, 72, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, 28, 72, 1);

    // SPD
    snprintf(buf, sizeof(buf), snap.speed > 0 ? "%d kt" : "---", snap.speed);
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("SPD", RX, 72, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, RX + 24, 72, 1);

    // HDG
    snprintf(buf, sizeof(buf), snap.heading > 0 ? "%d\xB0" : "---", snap.heading);
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("HDG", 4, 84, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, 28, 84, 1);

    // V/S
    if (snap.vert_rate != 0)
        snprintf(buf, sizeof(buf), "%+d fpm", (int)snap.vert_rate);
    else
        strlcpy(buf, "---", sizeof(buf));
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("V/S", RX, 84, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, RX + 24, 84, 1);

    // DST
    snprintf(buf, sizeof(buf), "%.1f nm", dist_val);
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("DST", 4, 96, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, 28, 96, 1);

    // SQK
    snprintf(buf, sizeof(buf), snap.squawk > 0 ? "%04d" : "---", snap.squawk);
    tft.setTextColor(pal->fade_dim, pal->bg); tft.drawString("SQK", RX, 96, 1);
    tft.setTextColor(pal->text, pal->bg);     tft.drawString(buf, RX + 24, 96, 1);

    tft.drawLine(0, 108, LCD_H_RES, 108, pal->grid);

    // ── Route ────────────────────────────────────────────────────────────

    snprintf(buf, sizeof(buf), "ORIG  %s", snap.origin[0] ? snap.origin : "---");
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(buf, 4, 112, 1);

    tft.setTextColor(pal->fade_dim, pal->bg);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(">", 160, 112, 1);

    snprintf(buf, sizeof(buf), "DEST  %s", snap.dest[0] ? snap.dest : "---");
    tft.setTextColor(pal->hdr, pal->bg);
    tft.setTextDatum(TR_DATUM);
    tft.drawString(buf, 316, 112, 1);

    // Full airport names from enrichment
    tft.setTextColor(pal->fade_med, pal->bg);
    if (enr && (enr->origin_airport[0] || enr->destination_airport[0])) {
        char oname[22] = ""; char dname[22] = "";
        strlcpy(oname, enr->origin_airport, sizeof(oname));
        strlcpy(dname, enr->destination_airport, sizeof(dname));
        tft.setTextDatum(TL_DATUM);
        if (oname[0]) tft.drawString(oname, 4, 124, 1);
        tft.setTextDatum(TR_DATUM);
        if (dname[0]) tft.drawString(dname, 316, 124, 1);
    } else {
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(enr && enr->loading ? "fetching route..." : "no route data", 160, 124, 1);
    }

    tft.drawLine(0, 136, LCD_H_RES, 136, pal->grid);

    // ── Aircraft Details ──────────────────────────────────────────────────

    if (enr && enr->loaded) {
        // MFGR + YEAR
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.drawString("MFGR", 4, 140, 1);
        tft.setTextColor(pal->text, pal->bg);
        tft.drawString(enr->manufacturer[0] ? enr->manufacturer : "---", 34, 140, 1);
        if (enr->year_built > 0) {
            snprintf(buf, sizeof(buf), "YEAR  %d", enr->year_built);
            tft.setTextColor(pal->fade_dim, pal->bg);
            tft.setTextDatum(TR_DATUM);
            tft.drawString(buf, 316, 140, 1);
        }

        // ENGS + CTRY
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.drawString("ENGS", 4, 152, 1);
        tft.setTextColor(pal->text, pal->bg);
        if (enr->engine_count > 0 && enr->engine_type[0])
            snprintf(buf, sizeof(buf), "%d x %s", enr->engine_count, enr->engine_type);
        else if (enr->engine_count > 0)
            snprintf(buf, sizeof(buf), "%d", enr->engine_count);
        else
            strlcpy(buf, "---", sizeof(buf));
        tft.drawString(buf, 34, 152, 1);
        if (enr->registered_country[0]) {
            snprintf(buf, sizeof(buf), "CTRY  %.16s", enr->registered_country);
            tft.setTextColor(pal->fade_dim, pal->bg);
            tft.setTextDatum(TR_DATUM);
            tft.drawString(buf, 316, 152, 1);
        }

        // OWNER
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.drawString("OWN", 4, 164, 1);
        tft.setTextColor(pal->text, pal->bg);
        const char *own = enr->owner[0] ? enr->owner :
                          (snap.owner_op[0] ? snap.owner_op : "---");
        tft.drawString(own, 28, 164, 1);

    } else {
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(enr && enr->loading ? "fetching details..." : "no detail data",
                       160, 155, 1);
    }

    tft.drawLine(0, 176, LCD_H_RES, 176, pal->grid);

    // ── Bottom: photo credit + back button ───────────────────────────────

    if (enr && enr->photo_photographer[0]) {
        tft.setTextColor(pal->fade_dim, pal->bg);
        tft.setTextDatum(TL_DATUM);
        snprintf(buf, sizeof(buf), "Photo: %.40s", enr->photo_photographer);
        tft.drawString(buf, 4, 195, 1);
    }

    tft.fillRect(291, 215, 28, 24, pal->grid);
    draw_radar_glyph(305, 227, pal->text);
}

// ---- Alert overlay ----

static void check_alerts() {
    if (!g_config.alert_military && !g_config.alert_emergency) return;

    if (aircraft_list.lock(pdMS_TO_TICKS(30))) {
        for (int i = 0; i < aircraft_list.count; i++) {
            Aircraft &a = aircraft_list.aircraft[i];

            bool is_alert = false;
            const char *type_str = "";
            uint16_t color = pal->blip;

            if (g_config.alert_military && a.is_military) {
                is_alert = true; type_str = "MIL"; color = pal->blip_mil;
            } else if (g_config.alert_emergency && a.is_emergency) {
                is_alert = true; type_str = "EMG"; color = pal->blip_emg;
            }

            if (!is_alert) continue;

            // Dedup check
            uint32_t h = hash_icao(a.icao_hex);
            bool dup = false;
            for (int d = 0; d < ALERT_DEDUP_SIZE; d++) {
                if (alerted_icaos[d] == h) { dup = true; break; }
            }
            if (dup) continue;

            // New alert
            alerted_icaos[alerted_idx] = h;
            alerted_idx = (alerted_idx + 1) % ALERT_DEDUP_SIZE;

            alert.active = true;
            alert.start_time = millis();
            alert.color = color;
            snprintf(alert.message, sizeof(alert.message), "%s %s", type_str, a.callsign[0] ? a.callsign : a.icao_hex);
            break;  // one alert at a time
        }
        aircraft_list.unlock();
    }
}

static void draw_alert_overlay() {
    if (!alert.active) return;

    uint32_t elapsed = millis() - alert.start_time;
    if (elapsed > ALERT_DURATION_MS) {
        alert.active = false;
        radar_needs_full_redraw = true;
        view_changed = true;
        return;
    }

    // Flashing border (toggle every 250ms)
    bool on = ((elapsed / 250) % 2) == 0;
    uint16_t border_color = on ? alert.color : pal->bg;

    tft.drawRect(0, STATUS_H, LCD_H_RES, RADAR_H, border_color);
    tft.drawRect(1, STATUS_H + 1, LCD_H_RES - 2, RADAR_H - 2, border_color);

    // Banner at top of view area
    if (on) {
        tft.fillRect(2, STATUS_H + 2, LCD_H_RES - 4, 16, alert.color);
        tft.setTextColor(pal->bg, alert.color);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(alert.message, LCD_H_RES / 2, STATUS_H + 10, 2);
    }
}

// ---- Closest approach + flight log update ----

static uint32_t last_once_per_sec = 0;

static void update_once_per_sec() {
    uint32_t now = millis();
    if (now - last_once_per_sec < 1000) return;
    last_once_per_sec = now;

    check_alerts();

    if (!aircraft_list.lock(pdMS_TO_TICKS(30))) return;

    for (int i = 0; i < aircraft_list.count; i++) {
        Aircraft &a = aircraft_list.aircraft[i];
        if (a.lat == 0.0f && a.lon == 0.0f) continue;
        if (compute_aircraft_opacity(a.stale_since, now) == 0) continue;

        float d = distance_nm(a.lat, a.lon);

        // Update closest approach record
        if (d < closest_record.distance_nm) {
            closest_record.distance_nm = d;
            closest_record.altitude = a.altitude;
            closest_record.timestamp = now;
            strlcpy(closest_record.callsign, a.callsign[0] ? a.callsign : a.icao_hex, sizeof(closest_record.callsign));
            strlcpy(closest_record.type_code, a.type_code, sizeof(closest_record.type_code));
        }

        // Flight log: check if already logged
        uint32_t h = hash_icao(a.icao_hex);
        bool found = false;
        for (int j = 0; j < log_count; j++) {
            int li = (log_write_idx - 1 - j + LOG_SIZE) % LOG_SIZE;
            if (flight_log[li].icao_hash == h) {
                // Update closest distance
                if (d < flight_log[li].closest_nm)
                    flight_log[li].closest_nm = d;
                found = true;
                break;
            }
        }
        if (!found) {
            LogEntry &e = flight_log[log_write_idx];
            e.icao_hash = h;
            strlcpy(e.callsign, a.callsign[0] ? a.callsign : a.icao_hex, sizeof(e.callsign));
            strlcpy(e.type_code, a.type_code, sizeof(e.type_code));
            e.first_seen = now;
            e.closest_nm = d;
            e.is_military = a.is_military;
            e.is_emergency = a.is_emergency;
            log_write_idx = (log_write_idx + 1) % LOG_SIZE;
            if (log_count < LOG_SIZE) log_count++;
        }
    }
    aircraft_list.unlock();
}

// ---- Auto-cycle ----

static void auto_cycle() {
    // Only radar and settings remain; nothing to auto-cycle between
}

// ---- Setup ----

void setup() {
    Serial.begin(115200);
    Serial.println("ADS-B CYD Radar starting...");

    // Backlight via PWM
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(TFT_BL, 5000, 8);  // 5kHz, 8-bit resolution
#else
    ledcSetup(0, 5000, 8);
    ledcAttachPin(TFT_BL, 0);
#endif

    // Display
    tft.init();
    tft.invertDisplay(true);
    tft.setRotation(1);

    tft.fillScreen(TFT_BLACK);
    status_spr.createSprite(LCD_H_RES, STATUS_H);

    // Touch
    touchSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    pinMode(XPT2046_IRQ, INPUT);
    pinMode(XPT2046_CS, OUTPUT);
    digitalWrite(XPT2046_CS, HIGH);

    // Data layer
    error_log_init();
    g_config = storage_load_config();
    aircraft_list.init();
    enrichment_init();
    fetcher_init(&aircraft_list);

    // Apply persisted settings
    pal = g_config.night_mode ? &PALETTE_NIGHT : &PALETTE_GREEN;
    apply_brightness();

    uint32_t now = millis();
    last_touch_time = now;
    last_cycle_time = now;

    Serial.printf("Aircraft array: %d x %d = %d bytes\n",
        MAX_AIRCRAFT, (int)sizeof(Aircraft), MAX_AIRCRAFT * (int)sizeof(Aircraft));
    Serial.printf("Free heap: %lu\n",
        (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

// ---- Main loop ----

static uint32_t last_draw = 0;
static bool touch_was_down = false;

void loop() {
    uint32_t now = millis();

    // Touch handling with long-press detection
    int tx, ty;
    if (getTouchPoint(tx, ty)) {
        if (!touch_was_down) {
            touch_was_down = true;
            touch_down_time = now;
            long_press_fired = false;
            saved_tx = tx;
            saved_ty = ty;
            last_touch_time = now;
            last_cycle_time = now;
        } else {
            saved_tx = tx;  // track latest position
            saved_ty = ty;
            if (!long_press_fired && (now - touch_down_time) >= LONG_PRESS_MS) {
                long_press_fired = true;
            }
        }
    } else {
        if (touch_was_down && !long_press_fired) {
            switch (current_view) {
                case VIEW_RADAR:    handle_touch_radar(saved_tx, saved_ty); break;
                case VIEW_INFO:     handle_touch_info(saved_tx, saved_ty); break;
                case VIEW_SETTINGS: handle_touch_settings(saved_tx, saved_ty); break;
                default: break;
            }
            draw_status_bar(true);
        }
        touch_was_down = false;
    }

    // Once-per-second updates (alerts, log, closest)
    update_once_per_sec();

    // Auto-cycle
    auto_cycle();

    // Redraw at ~15fps
    if (now - last_draw >= 66) {
        last_draw = now;

        if (current_view == VIEW_LOADING) {
            draw_loading();
            static uint32_t loading_done_time = 0;
            if (fetcher_last_update() > 0 && loading_done_time == 0) {
                loading_done_time = now;  // start brief pause to show count
            }
            if (loading_done_time > 0 && now - loading_done_time > 1200) {
                current_view = VIEW_RADAR;
                tft.fillScreen(pal->bg);
                radar_needs_full_redraw = true;
                draw_status_bar(true);
            }
        } else {
            sweep_angle += SWEEP_SPEED;
            if (sweep_angle >= 360.0f) sweep_angle -= 360.0f;

            draw_status_bar(false);
            switch (current_view) {
                case VIEW_RADAR:    draw_radar(); break;
                case VIEW_INFO:     draw_info(); break;
                case VIEW_SETTINGS: draw_settings(); break;
                default: break;
            }

            // Alert overlay on top
            draw_alert_overlay();
        }
    }

    enrichment_poll();
    delay(5);
}
