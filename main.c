/*
 * main_gui.c  –  Algae-Halt | Riskmonitor v4
 * Target: Windows (MinGW) + raylib
 *
 * Build:
 *   gcc main_gui.c -o algae_halt.exe -I./include -L./lib
 *       -lraylib -lopengl32 -lgdi32 -lwinmm -lm -O2
 *
 * Serial format from MCU:
 *   SENSOR pH=7.20 temp=23.45 Turbidity=4.2 Voltage=3.1
 *
 * Alla riskberäkningar sker här i GUI:n – MCU skickar bara rådata.
 *
 * Turbiditet: sensor är inverterad – hög mV = grumigt, låg mV = klart.
 *   NTU   = mV / 4500 * 3000   (linjär mappning)
 *   Turb_norm = clamp(NTU, 0, 3000) / 3000
 */

/* ── raylib måste inkluderas FÖRE windows.h ─────────────────────────────── */
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOGDI
#  define NOUSER
#  include <windows.h>
#  include <process.h>
#  ifdef DrawText
#    undef DrawText
#  endif
#  ifdef DrawRectangle
#    undef DrawRectangle
#  endif
#  ifdef CloseWindow
#    undef CloseWindow
#  endif
#  ifdef ShowCursor
#    undef ShowCursor
#  endif
#endif

/* ═══════════════════════════════════════════════════════════════════════════
   KONFIGURATION
═══════════════════════════════════════════════════════════════════════════ */
#define COM_PORT        "COM8"
#define COM_BAUD        115200
#define WIN_W           1100
#define WIN_H           680
#define MAX_HISTORY     128
#define GRAPH_POINTS    64

/* ═══════════════════════════════════════════════════════════════════════════
   FÄRGER  (mörkt terminal-tema)
═══════════════════════════════════════════════════════════════════════════ */
#define C_BG        (Color){ 13,  15,  14, 255}
#define C_BG2       (Color){ 20,  23,  21, 255}
#define C_BG3       (Color){ 28,  32,  29, 255}
#define C_BORDER    (Color){ 50,  58,  52, 255}
#define C_TEXT      (Color){210, 220, 212, 255}
#define C_DIM       (Color){100, 112, 103, 255}
#define C_MID       (Color){150, 165, 153, 255}
#define C_GREEN     (Color){ 61, 232, 122, 255}
#define C_YELLOW    (Color){245, 200,  66, 255}
#define C_ORANGE    (Color){240, 128,  48, 255}
#define C_RED       (Color){232,  64,  64, 255}
#define C_BLUE      (Color){ 55, 138, 221, 255}
#define C_PURPLE    (Color){127, 119, 221, 255}
#define C_ACCENT    (Color){ 61, 232, 122, 255}
#define C_WHITE     (Color){255, 255, 255, 255}

/* ═══════════════════════════════════════════════════════════════════════════
   DATASTRUKTURER
═══════════════════════════════════════════════════════════════════════════ */
typedef struct {
    float temp;        /* °C råvärde                */
    float ph;          /* pH råvärde                */
    float turb_mv;     /* mV råvärde 0–4500         */
    float turb_ntu;    /* beräknat NTU              */
    float t_norm;      /* normaliserad temp  0–1    */
    float ph_norm;     /* normaliserad pH    0–1    */
    float turb_norm;   /* normaliserad turb  0–1    */
    float risk;        /* sammansatt risk    0–1    */
    float delta_risk_h;/* ΔRisk/h vs föregående    */
    double ts;         /* tidsstämpel (sekunder)    */
    char  time_str[20];/* "HH:MM:SS"                */
} DataPoint;

/* ═══════════════════════════════════════════════════════════════════════════
   GLOBALT DELAT TILLSTÅND (serial-tråd ↔ render-loop)
═══════════════════════════════════════════════════════════════════════════ */
static volatile float g_ph        = 7.0f;
static volatile float g_temp      = 20.0f;
static volatile float g_turb_mv   = 0.0f;
static volatile int   g_has_turb  = 0;
static volatile int   g_temp_ok   = 1;
static volatile int   g_serial_ok = 0;
static volatile int   g_new_data  = 0;   /* sätts av serial-tråd, nollras av GUI */

#ifdef _WIN32
static CRITICAL_SECTION g_lock;

/* ── Serial-tråd ────────────────────────────────────────────────────────── */
static void parse_sensor_line(const char *line)
{
    if (strncmp(line, "SENSOR", 6) != 0) return;

    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    float ph = g_ph, temp = g_temp, turb_mv = g_turb_mv;
    int has_ph = 0, has_temp = 0, has_turb = 0, temp_none = 0;

    char *tok = strtok(buf, " \t\r\n");
    while (tok) {
        if      (strncmp(tok, "pH=",        3) == 0) { ph      = (float)atof(tok + 3); has_ph   = 1; }
        else if (strncmp(tok, "temp=",      5) == 0) {
            if  (strcmp(tok + 5, "NONE")    == 0)    { temp_none = 1; }
            else                                      { temp = (float)atof(tok + 5); has_temp = 1; }
        }
        else if (strncmp(tok, "Turbidity=", 10) == 0) { turb_mv = (float)atof(tok + 10); has_turb = 1; }
        tok = strtok(NULL, " \t\r\n");
    }

    EnterCriticalSection(&g_lock);
    if (has_ph)   { g_ph = ph; }
    if (has_temp) { g_temp = temp; g_temp_ok = 1; }
    if (temp_none){ g_temp_ok = 0; }
    if (has_turb) { g_turb_mv = turb_mv; g_has_turb = 1; }
    g_new_data = 1;
    LeaveCriticalSection(&g_lock);
}

static unsigned __stdcall serial_thread(void *arg)
{
    (void)arg;
    char port_path[32];
    snprintf(port_path, sizeof(port_path), "\\\\.\\%s", COM_PORT);

    HANDLE hPort = CreateFileA(port_path, GENERIC_READ, 0, NULL,
                               OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hPort == INVALID_HANDLE_VALUE) return 0;

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(hPort, &dcb);
    dcb.BaudRate = COM_BAUD; dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY; dcb.StopBits = ONESTOPBIT;
    SetCommState(hPort, &dcb);

    COMMTIMEOUTS ct = {50, 10, 100, 0, 0};
    SetCommTimeouts(hPort, &ct);

    EnterCriticalSection(&g_lock);
    g_serial_ok = 1;
    LeaveCriticalSection(&g_lock);

    char line[256]; int pos = 0; char ch = 0; DWORD rd = 0;
    while (1) {
        if (!ReadFile(hPort, &ch, 1, &rd, NULL) || rd == 0) continue;
        if (ch == '\n' || ch == '\r') {
            if (pos > 0) { line[pos] = '\0'; parse_sensor_line(line); pos = 0; }
        } else if (pos < (int)(sizeof(line) - 1)) {
            line[pos++] = ch;
        }
    }
    CloseHandle(hPort);
    return 0;
}
#endif /* _WIN32 */

/* ═══════════════════════════════════════════════════════════════════════════
   RISKBERÄKNINGAR
═══════════════════════════════════════════════════════════════════════════ */

/* mV → NTU: sensor är inverterad – hög mV = grumigt, låg mV = klart.
 * Linjär mappning: 0 mV = 0 NTU, 4500 mV = 3000 NTU */
static float mv_to_ntu(float mv)
{
    float ntu = (mv / 4500.0f) * 3000.0f;
    if (ntu < 0.0f)    ntu = 0.0f;
    if (ntu > 3000.0f) ntu = 3000.0f;
    return ntu;
}

/* Normalisera temp: riskzon 16–28°C */
static float norm_temp(float T)
{
    float v = (T - 16.0f);
    if (v < 0.0f)  v = 0.0f;
    if (v > 12.0f) v = 12.0f;
    return v / 12.0f;
}

/* Normalisera pH: riskzon 4.0–10.0 */
static float norm_ph(float pH)
{
    float v = (pH - 4.0f);
    if (v < 0.0f)  v = 0.0f;
    if (v > 6.0f)  v = 6.0f;
    return v / 6.0f;
}

/* Normalisera turbiditet: NTU 0–3000, hög NTU = hög risk */
static float norm_turb(float ntu)
{
    if (ntu > 3000.0f) ntu = 3000.0f;
    if (ntu < 0.0f)    ntu = 0.0f;
    return 1.0f - (ntu / 3000.0f);
}

/* Sammansatt riskpoäng */
static float calc_risk(float t_norm, float ph_norm, float turb_norm)
{
    return 0.50f * t_norm + 0.30f * ph_norm + 0.20f * turb_norm;
}

/* Fyll i en DataPoint från råvärden */
static void fill_datapoint(DataPoint *dp, float temp, float ph, float turb_mv, double ts)
{
    dp->temp       = temp;
    dp->ph         = ph;
    dp->turb_mv    = turb_mv;
    dp->turb_ntu   = mv_to_ntu(turb_mv);
    dp->t_norm     = norm_temp(temp);
    dp->ph_norm    = norm_ph(ph);
    dp->turb_norm  = norm_turb(dp->turb_ntu);
    dp->risk       = calc_risk(dp->t_norm, dp->ph_norm, dp->turb_norm);
    dp->ts         = ts;
    dp->delta_risk_h = 0.0f;

    /* Tidssträng från ts */
    time_t t = (time_t)ts;
    struct tm *tm_info = localtime(&t);
    strftime(dp->time_str, sizeof(dp->time_str), "%H:%M:%S", tm_info);
}

/* Riskfärg */
static Color risk_color(float r)
{
    if (r < 0.20f) return C_GREEN;
    if (r < 0.50f) return C_YELLOW;
    if (r < 0.75f) return C_ORANGE;
    return C_RED;
}

/* Risketikett */
static const char *risk_label(float r)
{
    if (r < 0.20f) return "LAG";
    if (r < 0.50f) return "MATTLIG";
    if (r < 0.75f) return "HOG";
    return "MYCKET HOG";
}

/* ═══════════════════════════════════════════════════════════════════════════
   HISTORIK-RING
═══════════════════════════════════════════════════════════════════════════ */
static DataPoint history[MAX_HISTORY];
static int       hist_count = 0;   /* antal registrerade punkter */
static int       hist_head  = 0;   /* nästa skrivposition (ring) */

static void history_push(DataPoint *dp)
{
    /* Beräkna ΔRisk/h mot föregående */
    if (hist_count > 0) {
        int prev_idx = (hist_head - 1 + MAX_HISTORY) % MAX_HISTORY;
        DataPoint *prev = &history[prev_idx];
        double dh = (dp->ts - prev->ts) / 3600.0;
        if (dh > 0.0) dp->delta_risk_h = (dp->risk - prev->risk) / (float)dh;
    }

    history[hist_head] = *dp;
    hist_head = (hist_head + 1) % MAX_HISTORY;
    if (hist_count < MAX_HISTORY) hist_count++;
}

/* Hämta historik i kronologisk ordning (index 0 = äldst) */
static DataPoint *history_get(int idx)
{
    if (idx < 0 || idx >= hist_count) return NULL;
    int real = (hist_head - hist_count + idx + MAX_HISTORY) % MAX_HISTORY;
    return &history[real];
}

/* ═══════════════════════════════════════════════════════════════════════════
   RITHJÄLPARE
═══════════════════════════════════════════════════════════════════════════ */

static void draw_panel(int x, int y, int w, int h)
{
    DrawRectangle(x, y, w, h, C_BG2);
    DrawRectangleLinesEx((Rectangle){(float)x,(float)y,(float)w,(float)h}, 1, C_BORDER);
}

static void draw_section_title(const char *title, int x, int y)
{
    DrawText("//", x, y, 12, C_ACCENT);
    DrawText(title, x + 20, y, 12, C_DIM);
}

/* Liten färgad badge */
static void draw_badge(const char *txt, int x, int y, Color bg, Color tc)
{
    int tw = MeasureText(txt, 11);
    DrawRectangle(x - 4, y - 2, tw + 8, 17, bg);
    DrawText(txt, x, y, 11, tc);
}

/* Segmenterad fylld bar (som originaldesignen) */
static void draw_seg_bar(int x, int y, int w, int h, float pct, Color col)
{
    DrawRectangle(x, y, w, h, C_BG3);
    DrawRectangleLinesEx((Rectangle){(float)x,(float)y,(float)w,(float)h}, 1, C_BORDER);

    int bsz = 10, gap = 2, pad = 3;
    int total  = (w - pad * 2) / (bsz + gap);
    int filled = (int)(total * pct);

    for (int i = 0; i < total; i++) {
        int bx = x + pad + i * (bsz + gap);
        int by = y + pad;
        int bh = h - pad * 2;
        if (i < filled) {
            DrawRectangle(bx, by, bsz, bh, col);
            DrawRectangle(bx, by, bsz, 2, (Color){255,255,255,50});
        } else {
            DrawRectangle(bx, by, bsz, bh, (Color){255,255,255,10});
        }
    }
}

/* Vertikalt hot-meter (höger sida) */
static void draw_threat_meter(int x, int y, int w, int h, float threat)
{
    draw_section_title("RISKNIVAINDIKATOR", x, y - 18);
    DrawRectangle(x, y, w, h, C_BG3);
    DrawRectangleLinesEx((Rectangle){(float)x,(float)y,(float)w,(float)h}, 1, C_BORDER);

    int bsz = 12, gap = 3, pad = 4;
    int total  = (h - pad * 2) / (bsz + gap);
    int filled = (int)(total * threat);

    for (int i = 0; i < total; i++) {
        int bx = x + pad;
        int by = y + h - pad - (i + 1) * (bsz + gap);
        int bw = w - pad * 2;

        if (i < filled) {
            float t = (float)i / (float)total;
            Color c;
            if      (t < 0.33f) c = C_GREEN;
            else if (t < 0.66f) c = C_YELLOW;
            else if (t < 0.85f) c = C_ORANGE;
            else                c = C_RED;
            DrawRectangle(bx, by, bw, bsz, c);
            DrawRectangle(bx, by, bw, 2, (Color){255,255,255,40});
        } else {
            DrawRectangle(bx, by, bw, bsz, (Color){255,255,255,8});
        }
    }

    Color tc = risk_color(threat);
    DrawText(TextFormat("%.0f%%", threat * 100.0f), x + 8, y + h + 8, 20, tc);
    DrawText(risk_label(threat), x - 2, y + h + 32, 11, tc);
}

/* Enkelt linjediagram */
static void draw_line_graph(int x, int y, int w, int h,
                             float *vals, int count,
                             float y_min, float y_max,
                             Color line_col, Color fill_col)
{
    if (count < 2) return;

    /* Fyllning under kurvan */
    for (int i = 0; i < count - 1; i++) {
        float x0 = x + (float)i / (count - 1) * w;
        float x1 = x + (float)(i + 1) / (count - 1) * w;
        float v0 = (vals[i]     - y_min) / (y_max - y_min);
        float v1 = (vals[i + 1] - y_min) / (y_max - y_min);
        if (v0 < 0) v0 = 0; if (v0 > 1) v0 = 1;
        if (v1 < 0) v1 = 0; if (v1 > 1) v1 = 1;
        float y0 = y + h - v0 * h;
        float y1 = y + h - v1 * h;

        /* Fyll med trapets */
        Vector2 pts[4] = {
            {x0, (float)(y + h)},
            {x0, y0},
            {x1, y1},
            {x1, (float)(y + h)}
        };
        DrawTriangle(pts[0], pts[1], pts[2], fill_col);
        DrawTriangle(pts[0], pts[2], pts[3], fill_col);
        DrawLineV((Vector2){x0, y0}, (Vector2){x1, y1}, line_col);
    }

    /* Punkter */
    for (int i = 0; i < count; i++) {
        float px = x + (float)i / (count - 1) * w;
        float v  = (vals[i] - y_min) / (y_max - y_min);
        if (v < 0) v = 0; if (v > 1) v = 1;
        float py = y + h - v * h;
        DrawCircle((int)px, (int)py, 3, line_col);
    }
}

/* Hjälinjor och axel för graf */
static void draw_graph_grid(int x, int y, int w, int h,
                             float y_min, float y_max, int steps)
{
    DrawRectangle(x, y, w, h, C_BG3);
    DrawRectangleLinesEx((Rectangle){(float)x,(float)y,(float)w,(float)h}, 1, C_BORDER);

    for (int s = 0; s <= steps; s++) {
        float v  = y_min + (y_max - y_min) * s / steps;
        int   gy = y + h - (int)((v - y_min) / (y_max - y_min) * h);
        DrawLine(x, gy, x + w, gy, (Color){255,255,255, s == 0 ? 30 : 12});
        DrawText(TextFormat("%.2f", v), x - 36, gy - 5, 9, C_DIM);
    }
}

/* ═══════════════════════════════════════════════════════════════════════════
   HUVUD-RENDER
═══════════════════════════════════════════════════════════════════════════ */

/* Sektioner vi kan visa */
typedef enum { VIEW_LIVE, VIEW_HISTORY, VIEW_FORMULAS } View;

int main(void)
{
#ifdef _WIN32
    InitializeCriticalSection(&g_lock);
    _beginthreadex(NULL, 0, serial_thread, NULL, 0, NULL);
#endif

    InitWindow(WIN_W, WIN_H, "Algae-Halt // Riskmonitor v4");
    SetTargetFPS(60);

    View  current_view   = VIEW_LIVE;
    float uptime         = 0.0f;
    float sample_timer   = 0.0f;
    float sample_interval = 2.0f;   /* sekunder mellan historik-punkter */

    /* Aktuella live-värden (uppdateras varje frame från serial) */
    float live_temp    = 20.0f;
    float live_ph      = 7.0f;
    float live_turb_mv = 0.0f;
    int   live_valid   = 0;

    /* Scroll för historiktabell */
    int   hist_scroll  = 0;
    int   rows_visible = 8;

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();
        uptime  += dt;

        /* ── Hämta live-data från serial-tråd ─────────────────────────────── */
#ifdef _WIN32
        EnterCriticalSection(&g_lock);
        live_temp    = g_temp;
        live_ph      = g_ph;
        live_turb_mv = g_turb_mv;
        live_valid   = g_serial_ok;
        int new_data = g_new_data;
        g_new_data   = 0;
        LeaveCriticalSection(&g_lock);
#else
        /* Demo-läge utan Windows */
        live_temp    = 20.0f + 5.0f  * sinf(uptime * 0.05f);
        live_ph      = 8.0f  + 0.8f  * sinf(uptime * 0.03f);
        live_turb_mv = 2000.0f + 1500.0f * sinf(uptime * 0.04f);
        live_valid   = 1;
        int new_data = 1;
#endif

        /* ── Spara punkt i historik varje sample_interval ────────────────── */
        sample_timer += dt;
        if (new_data && live_valid && sample_timer >= sample_interval) {
            sample_timer = 0.0f;
            DataPoint dp;
            fill_datapoint(&dp, live_temp, live_ph, live_turb_mv,
                           (double)uptime);
            history_push(&dp);
        }

        /* ── Beräkna aktuell risk ─────────────────────────────────────────── */
        float live_ntu      = mv_to_ntu(live_turb_mv);
        float live_tn       = norm_temp(live_temp);
        float live_phn      = norm_ph(live_ph);
        float live_turbn    = norm_turb(live_ntu);
        float live_risk     = calc_risk(live_tn, live_phn, live_turbn);
        Color live_rc       = risk_color(live_risk);
        const char *live_rl = risk_label(live_risk);

        /* ── Hämta peak och senaste delta ─────────────────────────────────── */
        float peak_risk   = 0.0f;
        float latest_delta = 0.0f;
        float latest_delta_h = 0.0f;
        for (int i = 0; i < hist_count; i++) {
            DataPoint *dp = history_get(i);
            if (dp->risk > peak_risk) peak_risk = dp->risk;
        }
        if (hist_count >= 2) {
            DataPoint *last = history_get(hist_count - 1);
            DataPoint *prev = history_get(hist_count - 2);
            latest_delta   = last->risk - prev->risk;
            double dh = (last->ts - prev->ts) / 3600.0;
            if (dh > 0.0) latest_delta_h = (float)(latest_delta / dh);
        }

        /* ── Bygg grafdata (senaste GRAPH_POINTS punkter) ────────────────── */
        int   gcount = hist_count < GRAPH_POINTS ? hist_count : GRAPH_POINTS;
        float g_risk[GRAPH_POINTS],
              g_tn[GRAPH_POINTS],
              g_phn[GRAPH_POINTS],
              g_turbn[GRAPH_POINTS];

        int start = hist_count - gcount;
        for (int i = 0; i < gcount; i++) {
            DataPoint *dp = history_get(start + i);
            g_risk[i]  = dp->risk;
            g_tn[i]    = dp->t_norm;
            g_phn[i]   = dp->ph_norm;
            g_turbn[i] = dp->turb_norm;
        }

        /* ════════════════════════════════════════════════════════════════════
           RITNING
        ════════════════════════════════════════════════════════════════════ */
        BeginDrawing();
        ClearBackground(C_BG);

        /* Rutnätsbakgrund */
        for (int gx = 0; gx < WIN_W; gx += 40)
            DrawLine(gx, 0, gx, WIN_H, (Color){255,255,255,5});
        for (int gy = 0; gy < WIN_H; gy += 40)
            DrawLine(0, gy, WIN_W, gy, (Color){255,255,255,5});

        /* ── TOPBAR ──────────────────────────────────────────────────────── */
        DrawRectangle(0, 0, WIN_W, 52, C_BG2);
        DrawLine(0, 52, WIN_W, 52, C_BORDER);

        DrawText("ALGAE-HALT", 20, 10, 22, C_ACCENT);
        DrawText("// RISKMONITOR v4", 130, 14, 13, C_DIM);
        DrawText("LAKE MALAREN - NODE 01", 20, 36, 11, C_DIM);

        /* Serial-status */
        Color sig_col = live_valid ? C_GREEN : C_RED;
        DrawCircle(WIN_W - 130, 20, 4, sig_col);
        DrawText(live_valid ? "LIVE " COM_PORT : "NO SIGNAL " COM_PORT,
                 WIN_W - 120, 14, 11, sig_col);
        DrawText(TextFormat("UPTIME: %.0fs", uptime), WIN_W - 120, 32, 10, C_DIM);

        /* ── NAVIGATIONSFLIKAR ───────────────────────────────────────────── */
        const char *tabs[] = {"  LIVE  ", "  HISTORIK  ", "  FORMLER  "};
        int tx = 340;
        for (int t = 0; t < 3; t++) {
            int tw = MeasureText(tabs[t], 12) + 10;
            Color tbg = (current_view == (View)t) ? C_BG3 : C_BG2;
            Color tfc = (current_view == (View)t) ? C_ACCENT : C_DIM;
            DrawRectangle(tx, 28, tw, 24, tbg);
            if (current_view == (View)t)
                DrawRectangle(tx, 50, tw, 2, C_ACCENT);
            DrawText(tabs[t], tx + 5, 34, 12, tfc);
            if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
                Vector2 mp = GetMousePosition();
                if (mp.x >= tx && mp.x <= tx + tw && mp.y >= 28 && mp.y <= 52)
                    current_view = (View)t;
            }
            tx += tw + 4;
        }

        /* ════════════════════════════════════════════════════════════════════
           VY: LIVE
        ════════════════════════════════════════════════════════════════════ */
        if (current_view == VIEW_LIVE)
        {
            int main_x = 20, main_y = 62, main_w = 820, panel_h = WIN_H - 70;
            int right_x = 850, right_w = WIN_W - 860;

            /* ── METRIKKORT (övre rad) ─────────────────────────────────── */
            int card_y = main_y + 4;
            int card_w = 188;
            int cards_x[] = {main_x, main_x+196, main_x+392, main_x+588};
            const char *card_labels[] = {"AKTUELL RISK", "DELTA RISK/H", "MATTNINGAR", "TOPPVARDE"};

            for (int c = 0; c < 4; c++) {
                draw_panel(cards_x[c], card_y, card_w, 72);
                /* Färgad toppkant */
                Color top_c = (c == 0) ? live_rc : C_ACCENT;
                DrawRectangle(cards_x[c], card_y, card_w, 2, top_c);
                DrawText(card_labels[c], cards_x[c] + 10, card_y + 8, 9, C_DIM);
            }

            /* Risk */
            DrawText(TextFormat("%.2f", live_risk),
                     cards_x[0] + 10, card_y + 22, 28, live_rc);
            draw_badge(live_rl, cards_x[0] + 10, card_y + 54, (Color){(unsigned char)(live_rc.r/5),(unsigned char)(live_rc.g/5),(unsigned char)(live_rc.b/5),200}, live_rc);

            /* Delta */
            Color dc = latest_delta_h > 0.001f ? C_RED :
                       latest_delta_h < -0.001f ? C_GREEN : C_DIM;
            DrawText(TextFormat("%+.4f/h", latest_delta_h),
                     cards_x[1] + 10, card_y + 26, 16, dc);
            DrawText(hist_count > 1 ? "vs foregaende" : "forsta matning",
                     cards_x[1] + 10, card_y + 52, 9, C_DIM);

            /* Mätningar */
            DrawText(TextFormat("%d", hist_count),
                     cards_x[2] + 10, card_y + 22, 28, C_TEXT);
            DrawText("registrerade", cards_x[2] + 10, card_y + 54, 9, C_DIM);

            /* Peak */
            DrawText(TextFormat("%.2f", peak_risk),
                     cards_x[3] + 10, card_y + 22, 28, risk_color(peak_risk));
            DrawText("max noterat", cards_x[3] + 10, card_y + 54, 9, C_DIM);

            /* ── PARAMETERBARS ─────────────────────────────────────────── */
            int bar_y = card_y + 84;
            int bar_w = 510;

            /* Temp */
            draw_panel(main_x, bar_y, bar_w + 130, 52);
            DrawText("TEMPERATUR", main_x + 10, bar_y + 6, 10, C_DIM);
            DrawText(TextFormat("%.1f C", live_temp), main_x + 120, bar_y + 5, 10, C_TEXT);
            DrawText(TextFormat("norm: %.3f", live_tn), main_x + 175, bar_y + 5, 9, C_DIM);
            draw_seg_bar(main_x + 10, bar_y + 22, bar_w, 22, live_tn, C_RED);
            DrawText("16", main_x + 10, bar_y + 46, 9, C_DIM);
            DrawText("28°C", main_x + 10 + bar_w - 30, bar_y + 46, 9, C_DIM);

            /* pH */
            bar_y += 60;
            draw_panel(main_x, bar_y, bar_w + 130, 52);
            DrawText("PH", main_x + 10, bar_y + 6, 10, C_DIM);
            DrawText(TextFormat("%.2f", live_ph), main_x + 120, bar_y + 5, 10, C_TEXT);
            DrawText(TextFormat("norm: %.3f", live_phn), main_x + 175, bar_y + 5, 9, C_DIM);
            draw_seg_bar(main_x + 10, bar_y + 22, bar_w, 22, live_phn, C_BLUE);
            DrawText("4.0", main_x + 10, bar_y + 46, 9, C_DIM);
            DrawText("10.0", main_x + 10 + bar_w - 26, bar_y + 46, 9, C_DIM);

            /* Turbiditet */
            bar_y += 60;
            draw_panel(main_x, bar_y, bar_w + 130, 52);
            DrawText("TURBIDITET", main_x + 10, bar_y + 6, 10, C_DIM);
            DrawText(TextFormat("%.0f mV -> %.1f NTU", live_turb_mv, live_ntu),
                     main_x + 120, bar_y + 5, 10, C_TEXT);
            DrawText(TextFormat("norm: %.3f", live_turbn), main_x + 320, bar_y + 5, 9, C_DIM);
            draw_seg_bar(main_x + 10, bar_y + 22, bar_w, 22, live_turbn, C_GREEN);
            DrawText("0 NTU", main_x + 10, bar_y + 46, 9, C_DIM);
            DrawText("3000 NTU", main_x + 10 + bar_w - 46, bar_y + 46, 9, C_DIM);

            /* ── RISKUTVECKLING GRAF ──────────────────────────────────── */
            int graph1_y = bar_y + 70;
            int graph1_h = 110;
            draw_panel(main_x, graph1_y, 780, graph1_h + 30);
            draw_section_title("RISKUTVECKLING OVER TID", main_x + 10, graph1_y + 8);

            int gx2 = main_x + 45, gy2 = graph1_y + 26;
            int gw2 = 700, gh2 = graph1_h - 10;
            draw_graph_grid(gx2, gy2, gw2, gh2, 0.0f, 1.0f, 4);
            if (gcount >= 2)
                draw_line_graph(gx2, gy2, gw2, gh2, g_risk, gcount,
                                0.0f, 1.0f, C_PURPLE,
                                (Color){127,119,221,25});

            /* ── PARAMETRAR GRAF ─────────────────────────────────────── */
            int graph2_y = graph1_y + graph1_h + 40;
            int graph2_h = 100;
            draw_panel(main_x, graph2_y, 780, graph2_h + 30);
            draw_section_title("PARAMETRAR OVER TID (NORMALISERADE)", main_x + 10, graph2_y + 8);

            int gx3 = main_x + 45, gy3 = graph2_y + 26;
            int gw3 = 700, gh3 = graph2_h - 10;
            draw_graph_grid(gx3, gy3, gw3, gh3, 0.0f, 1.0f, 4);
            if (gcount >= 2) {
                draw_line_graph(gx3, gy3, gw3, gh3, g_tn,    gcount, 0.0f, 1.0f,
                                C_RED,   (Color){232,64,64,0});
                draw_line_graph(gx3, gy3, gw3, gh3, g_phn,   gcount, 0.0f, 1.0f,
                                C_BLUE,  (Color){55,138,221,0});
                draw_line_graph(gx3, gy3, gw3, gh3, g_turbn, gcount, 0.0f, 1.0f,
                                C_GREEN, (Color){61,232,122,0});
            }
            /* Legend */
            int lx = gx3, ly = gy3 + gh3 + 6;
            DrawRectangle(lx,      ly, 10, 3, C_RED);   DrawText("Temp",  lx + 14, ly - 3, 10, C_DIM);
            DrawRectangle(lx + 60, ly, 10, 3, C_BLUE);  DrawText("pH",    lx + 74, ly - 3, 10, C_DIM);
            DrawRectangle(lx +110, ly, 10, 3, C_GREEN); DrawText("Turb",  lx +124, ly - 3, 10, C_DIM);

            /* ── HOT-METER (höger) ───────────────────────────────────── */
            int tm_h = WIN_H - 140;
            draw_threat_meter(right_x, 80, right_w, tm_h, live_risk);

            /* Risk-status längst ner */
            int status_y = WIN_H - 30;
            DrawText(live_risk > 0.75f ? "STATUS: FARA" :
                     live_risk > 0.50f ? "STATUS: VARNING" :
                     live_risk > 0.20f ? "STATUS: FORSIKTIGHET" :
                                         "STATUS: NORMALT",
                     20, status_y, 14, live_rc);
        }

        /* ════════════════════════════════════════════════════════════════════
           VY: HISTORIK
        ════════════════════════════════════════════════════════════════════ */
        else if (current_view == VIEW_HISTORY)
        {
            draw_panel(20, 62, WIN_W - 40, WIN_H - 70);
            draw_section_title("MATHISTORIK", 32, 72);

            /* Tabell-header */
            int tx2 = 30, ty = 94;
            const char *hdrs[] = {"TID","TEMP°C","PH","TURB mV","NTU","T_NORM","pH_NORM","TURB_NORM","RISK","NIVA","DR/H"};
            int col_x[] = {30,100,160,215,280,340,400,465,535,590,680};
            for (int c = 0; c < 11; c++)
                DrawText(hdrs[c], col_x[c], ty, 9, C_DIM);

            DrawLine(tx2, ty + 14, WIN_W - 30, ty + 14, C_BORDER);

            /* Scroll med mushjul */
            float wheel = GetMouseWheelMove();
            hist_scroll -= (int)wheel * 2;
            if (hist_scroll < 0) hist_scroll = 0;
            int max_scroll = hist_count - rows_visible;
            if (max_scroll < 0) max_scroll = 0;
            if (hist_scroll > max_scroll) hist_scroll = max_scroll;

            int row_y = ty + 18;
            int draw_from = hist_count - 1 - hist_scroll;

            for (int r = 0; r < rows_visible && draw_from - r >= 0; r++) {
                DataPoint *dp = history_get(draw_from - r);
                if (!dp) break;
                int ry = row_y + r * 26;
                if (r % 2 == 0)
                    DrawRectangle(30, ry - 2, WIN_W - 60, 24, (Color){255,255,255,4});

                Color rc = risk_color(dp->risk);
                DrawText(dp->time_str,                          col_x[0],  ry, 11, C_TEXT);
                DrawText(TextFormat("%.1f",  dp->temp),         col_x[1],  ry, 11, C_TEXT);
                DrawText(TextFormat("%.2f",  dp->ph),           col_x[2],  ry, 11, C_TEXT);
                DrawText(TextFormat("%.0f",  dp->turb_mv),      col_x[3],  ry, 11, C_TEXT);
                DrawText(TextFormat("%.1f",  dp->turb_ntu),     col_x[4],  ry, 11, C_TEXT);
                DrawText(TextFormat("%.3f",  dp->t_norm),       col_x[5],  ry, 11, C_MID);
                DrawText(TextFormat("%.3f",  dp->ph_norm),      col_x[6],  ry, 11, C_MID);
                DrawText(TextFormat("%.3f",  dp->turb_norm),    col_x[7],  ry, 11, C_MID);
                DrawText(TextFormat("%.3f",  dp->risk),         col_x[8],  ry, 12, rc);
                draw_badge(risk_label(dp->risk), col_x[9], ry,
                           (Color){(unsigned char)(rc.r/6),(unsigned char)(rc.g/6),(unsigned char)(rc.b/6),180}, rc);

                Color dc2 = dp->delta_risk_h > 0.001f ? C_RED :
                            dp->delta_risk_h < -0.001f ? C_GREEN : C_DIM;
                DrawText(dp->delta_risk_h != 0.0f ?
                         TextFormat("%+.4f", dp->delta_risk_h) : "—",
                         col_x[10], ry, 11, dc2);
            }

            /* Scrollbar */
            if (hist_count > rows_visible) {
                int sb_x = WIN_W - 22, sb_y = 110, sb_h = WIN_H - 130;
                DrawRectangle(sb_x, sb_y, 6, sb_h, C_BG3);
                float ratio  = (float)rows_visible / hist_count;
                float pos_r  = (float)hist_scroll  / hist_count;
                DrawRectangle(sb_x, sb_y + (int)(pos_r * sb_h),
                              6, (int)(ratio * sb_h), C_ACCENT);
            }

            DrawText(TextFormat("%d matpunkter  |  scrolla med mushjulet", hist_count),
                     30, WIN_H - 20, 10, C_DIM);
        }

        /* ════════════════════════════════════════════════════════════════════
           VY: FORMLER
        ════════════════════════════════════════════════════════════════════ */
        else if (current_view == VIEW_FORMULAS)
        {
            draw_panel(20, 62, WIN_W - 40, WIN_H - 70);
            draw_section_title("BERAKNINGSREFERENS", 32, 72);

            int fy = 96, fx = 34, fw = (WIN_W - 60) / 2 - 10;

            /* Block 1: Temp */
            draw_panel(fx, fy, fw, 90);
            DrawRectangle(fx, fy, fw, 2, C_RED);
            DrawText("1 · TEMPERATUR-NORMALISERING", fx+10, fy+8, 10, C_DIM);
            DrawText("Riskzon: 16 - 28 C", fx+10, fy+24, 11, C_TEXT);
            DrawText("T_norm = min(max(T - 16, 0), 12) / 12", fx+10, fy+40, 11, C_ACCENT);
            DrawText("Under 16C = 0 risk, over 28C = max risk", fx+10, fy+58, 10, C_DIM);
            DrawText(TextFormat("  Aktuellt: %.1fC -> %.3f", live_temp, live_tn), fx+10, fy+72, 10, C_MID);

            /* Block 2: pH */
            draw_panel(fx + fw + 14, fy, fw, 90);
            DrawRectangle(fx + fw + 14, fy, fw, 2, C_BLUE);
            DrawText("2 · PH-NORMALISERING", fx+fw+24, fy+8, 10, C_DIM);
            DrawText("Riskzon: 4.0 - 10.0", fx+fw+24, fy+24, 11, C_TEXT);
            DrawText("pH_norm = min(max(pH - 4.0, 0), 6.0) / 6.0", fx+fw+24, fy+40, 11, C_ACCENT);
            DrawText("Under 4.0 = 0 risk, over 10.0 = max risk", fx+fw+24, fy+58, 10, C_DIM);
            DrawText(TextFormat("  Aktuellt: %.2f -> %.3f", live_ph, live_phn), fx+fw+24, fy+72, 10, C_MID);

            /* Block 3: Turbiditet */
            fy += 104;
            draw_panel(fx, fy, fw, 110);
            DrawRectangle(fx, fy, fw, 2, C_GREEN);
            DrawText("3 · TURBIDITET (mV -> NTU -> norm)", fx+10, fy+8, 10, C_DIM);
            DrawText("Sensor: 0-4500 mV  (HOG mV = GRUMIGT)", fx+10, fy+24, 11, C_TEXT);
            DrawText("Linjar: NTU = mV / 4500 * 3000", fx+10, fy+40, 11, C_ACCENT);
            DrawText("0 mV = 0 NTU (klart),  4500 mV = 3000 NTU", fx+10, fy+54, 11, C_ACCENT);
            DrawText("Turb_norm = clamp(NTU, 0, 3000) / 3000", fx+10, fy+68, 11, C_ACCENT);
            DrawText(TextFormat("  Aktuellt: %.0f mV -> %.1f NTU -> %.3f",
                     live_turb_mv, live_ntu, live_turbn), fx+10, fy+84, 10, C_MID);

            /* Block 4: Risk */
            draw_panel(fx + fw + 14, fy, fw, 110);
            DrawRectangle(fx + fw + 14, fy, fw, 2, C_YELLOW);
            DrawText("4 · SAMMANSATT RISKPOANG", fx+fw+24, fy+8, 10, C_DIM);
            DrawText("Viktad linjar kombination  (0-1)", fx+fw+24, fy+24, 11, C_TEXT);
            DrawText("Risk = 0.50 * T_norm", fx+fw+24, fy+40, 11, C_ACCENT);
            DrawText("     + 0.30 * pH_norm", fx+fw+24, fy+54, 11, C_ACCENT);
            DrawText("     + 0.20 * Turb_norm", fx+fw+24, fy+68, 11, C_ACCENT);
            DrawText("Temp vagast (50%), pH (30%), Turb (20%)", fx+fw+24, fy+86, 10, C_DIM);
            DrawText(TextFormat("  Aktuellt: %.3f  (%s)",
                     live_risk, live_rl), fx+fw+24, fy+100, 10, live_rc);

            /* Block 5: Förändringshastighet */
            fy += 124;
            draw_panel(fx, fy, fw, 90);
            DrawRectangle(fx, fy, fw, 2, C_PURPLE);
            DrawText("5 · FORANDRINGSTAKT  DR/h", fx+10, fy+8, 10, C_DIM);
            DrawText("Hur snabbt risken okar eller minskar", fx+10, fy+24, 11, C_TEXT);
            DrawText("dR/h = (Risk_n - Risk_(n-1)) / dt_h", fx+10, fy+40, 11, C_ACCENT);
            DrawText("Positiv = okande risk, negativ = sjunkande", fx+10, fy+58, 10, C_DIM);
            DrawText(TextFormat("  Senaste: %+.4f/h", latest_delta_h), fx+10, fy+72, 10, C_MID);

            /* Block 6: Tidsviktad */
            draw_panel(fx + fw + 14, fy, fw, 90);
            DrawRectangle(fx + fw + 14, fy, fw, 2, C_ORANGE);
            DrawText("6 · TIDSVIKTAD DR/h  (LARM)", fx+fw+24, fy+8, 10, C_DIM);
            DrawText("Straffar langa tidsintervall", fx+fw+24, fy+24, 11, C_TEXT);
            DrawText("dR_vikt = (dR/dt) * (1h / max(dt, 1h))", fx+fw+24, fy+40, 11, C_ACCENT);
            DrawText("Larm om vikt > 0.15 (snabb okning)", fx+fw+24, fy+58, 10, C_DIM);
            DrawText("Forsiktighet om vikt > 0.05", fx+fw+24, fy+72, 10, C_DIM);
        }

        /* ── LARM (alltid synligt längst ner om risk hög) ────────────────── */
        if (live_risk > 0.75f) {
            DrawRectangle(0, WIN_H - 22, WIN_W, 22, (Color){232,64,64,40});
            DrawLine(0, WIN_H - 22, WIN_W, WIN_H - 22, C_RED);
            DrawText("!! HOG BLOMNINGSRISK – KONTROLLERA VATTENPROVET OMEDELBART !!",
                     WIN_W/2 - 250, WIN_H - 17, 11, C_RED);
        } else if (live_risk > 0.50f && current_view == VIEW_LIVE) {
            DrawRectangle(0, WIN_H - 22, WIN_W, 22, (Color){240,128,48,25});
            DrawLine(0, WIN_H - 22, WIN_W, WIN_H - 22, C_ORANGE);
            DrawText("VARNING: Mattlig blomningsrisk – overvaka noggrant",
                     WIN_W/2 - 190, WIN_H - 17, 11, C_ORANGE);
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
