/*
 * main_gui.c  –  Algae-Halt | Water Quality Monitor
 *
 * Original design preserved exactly.
 * Serial thread (COM8, 115200) updates ph_level and tmp_level from MCU.
 * Expected line format: SENSOR pH=7.2 temp=23.45\r\n
 *
 * Build (Windows, MinGW):
 *   gcc main_gui.c -o algae_halt.exe -I./include -L./lib
 *       -lraylib -lopengl32 -lgdi32 -lwinmm -O2
 */

/* ── CRITICAL: raylib MUST come before windows.h ──────────────────────────
   windows.h redefines DrawText, DrawLine, Color etc. as GDI macros which
   causes all 22 "incompatible type" errors. Fix:
     1. Include raylib first so its types are already declared.
     2. WIN32_LEAN_AND_MEAN removes most GDI/User32 clashes.
     3. NOGDI removes the remaining drawing-function macros.
     4. #undef the handful that still sneak through.              */
#include "raylib.h"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOGDI
#  define NOUSER
#  include <windows.h>
#  include <process.h>
/* Undef anything windows.h still snuck in */
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

/* ── COM port ────────────────────────────────────────────────────────────── */
#define COM_PORT "COM8"
#define COM_BAUD 115200

/* ── Shared state (written by serial thread, read by render loop) ─────────── */
static volatile float g_ph       = 7.0f;
static volatile float g_temp     = 20.0f;
static volatile int   g_temp_ok  = 1;
static volatile int   g_serial_ok = 0;

#ifdef _WIN32
static CRITICAL_SECTION g_lock;

static void parse_sensor_line(const char *line)
{
    if (strncmp(line, "SENSOR", 6) != 0) return;

    char buf[128];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    float ph = g_ph, temp = g_temp;
    int has_ph = 0, has_temp = 0, temp_none = 0;

    char *tok = strtok(buf, " \t\r\n");
    while (tok) {
        if      (strncmp(tok, "pH=",   3) == 0) { ph   = (float)atof(tok + 3); has_ph   = 1; }
        else if (strncmp(tok, "temp=", 5) == 0) {
            if (strcmp(tok + 5, "NONE") == 0)   { temp_none = 1; }
            else                                 { temp = (float)atof(tok + 5); has_temp = 1; }
        }
        tok = strtok(NULL, " \t\r\n");
    }

    if (has_ph) {
        EnterCriticalSection(&g_lock);
        g_ph      = ph;
        g_temp_ok = has_temp && !temp_none;
        if (has_temp) g_temp = temp;
        LeaveCriticalSection(&g_lock);
    }
}

static unsigned __stdcall serial_thread(void *arg)
{
    (void)arg;
    HANDLE hPort = CreateFileA("\\\\.\\" COM_PORT, GENERIC_READ, 0, NULL,
                               OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (hPort == INVALID_HANDLE_VALUE) return 0;

    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(hPort, &dcb);
    dcb.BaudRate = COM_BAUD; dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;   dcb.StopBits = ONESTOPBIT;
    SetCommState(hPort, &dcb);

    COMMTIMEOUTS ct = {50, 10, 100, 0, 0};
    SetCommTimeouts(hPort, &ct);
    g_serial_ok = 1;

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
   ORIGINAL DESIGN — untouched from your main.c
   ph_level and tmp_level are set from serial each frame instead of buttons.
   al_level and trb_level still use the +/- buttons (no sensor yet).
   ═══════════════════════════════════════════════════════════════════════════ */

void draw_sensor_bar(const char *label, const char *unit, int *level, int maxLevel,
                     float *timer, int y)
{
    if (*timer > 0.0f) {
        *timer -= GetFrameTime();
        *level += 1;
    }
    if (*level < 0)        *level = 0;
    if (*level > maxLevel) *level = maxLevel;

    float pct = (float)*level / (float)maxLevel;

    Color barColor;
    if      (pct < 0.4f) barColor = GREEN;
    else if (pct < 0.7f) barColor = YELLOW;
    else                 barColor = RED;

    DrawText(label,                              50,  y - 22, 18, (Color){180, 180, 200, 255});
    DrawText(TextFormat("%d %s", *level, unit), 460, y - 22, 18, barColor);

    DrawRectangleLinesEx((Rectangle){50, y, 430, 36}, 2, (Color){180, 180, 180, 200});
    DrawRectangle(51, y + 1, 428, 34, (Color){20, 20, 30, 255});

    int blockSize = 12, gap = 3, padding = 4;
    int totalBlocks  = (430 - padding * 2) / (blockSize + gap);
    int filledBlocks = (int)(totalBlocks * pct);

    for (int i = 0; i < totalBlocks; i++) {
        int bx = 50 + padding + i * (blockSize + gap);
        int by = y  + padding;
        int bh = 36 - padding * 2;
        if (i < filledBlocks) {
            DrawRectangle(bx, by, blockSize, bh, barColor);
            DrawRectangle(bx, by, blockSize, 3,  (Color){255, 255, 255, 60});
        } else {
            DrawRectangle(bx, by, blockSize, bh, (Color){255, 255, 255, 12});
        }
    }

    if (GuiButton((Rectangle){490, y, 36, 36}, "+") && *level < maxLevel)
        *timer = 0.5f;
    if (GuiButton((Rectangle){532, y, 36, 36}, "-") && *level > 0) {
        *level -= 10;
        if (*level < 0) *level = 0;
    }
}

void draw_threat_meter(float threat)
{
    int x = 660, y = 80, w = 100, h = 390;
    DrawText("TOTAL",  x + 10, y - 55, 18, WHITE);
    DrawText("THREAT", x + 5,  y - 35, 18, WHITE);

    DrawRectangleLinesEx((Rectangle){x, y, w, h}, 2, (Color){180, 180, 180, 200});
    DrawRectangle(x + 1, y + 1, w - 2, h - 2, (Color){20, 20, 30, 255});

    int blockSize = 14, gap = 3, padding = 5;
    int totalBlocks  = (h - padding * 2) / (blockSize + gap);
    int filledBlocks = (int)(totalBlocks * threat);

    for (int i = 0; i < totalBlocks; i++) {
        int bx = x + padding;
        int by = y + h - padding - (i + 1) * (blockSize + gap);
        int bw = w - padding * 2;

        if (i < filledBlocks) {
            float t = (float)i / totalBlocks;
            Color c;
            if (t < 0.5f) c = (Color){(unsigned char)(t * 2 * 255), 200, 50, 255};
            else           c = (Color){255, (unsigned char)((1.0f - (t - 0.5f) * 2) * 200), 50, 255};
            DrawRectangle(bx, by, bw, blockSize, c);
            DrawRectangle(bx, by, bw, 3, (Color){255, 255, 255, 50});
        } else {
            DrawRectangle(bx, by, bw, blockSize, (Color){255, 255, 255, 12});
        }
    }

    Color tc = threat > 0.7f ? RED : threat > 0.4f ? YELLOW : GREEN;
    DrawText(TextFormat("%.0f%%", threat * 100), x + 22, y + h + 10, 22, tc);
}

int main(void)
{
#ifdef _WIN32
    InitializeCriticalSection(&g_lock);
    _beginthreadex(NULL, 0, serial_thread, NULL, 0, NULL);
#endif

    InitWindow(800, 520, "Algae-Halt | Water Monitor");
    SetTargetFPS(60);

    int   ph_level  = 30; float ph_timer  = 0.0f;
    int   al_level  = 50; float al_timer  = 0.0f;
    int   tmp_level = 20; float tmp_timer = 0.0f;
    int   trb_level = 10; float trb_timer = 0.0f;
    float time      = 0.0f;

    while (!WindowShouldClose())
    {
        time += GetFrameTime();

        /* ── Pull live values from serial thread ──────────────────────────── */
#ifdef _WIN32
        EnterCriticalSection(&g_lock);
        float cur_ph    = g_ph;
        float cur_temp  = g_temp;
        int   temp_ok   = g_temp_ok;
        int   serial_ok = g_serial_ok;
        LeaveCriticalSection(&g_lock);
#else
        /* Non-Windows demo mode */
        float cur_ph    = 7.25f + 1.25f * sinf(time * 0.1f);
        float cur_temp  = 22.5f + 7.5f  * sinf(time * 0.07f);
        int   temp_ok   = 1;
        int   serial_ok = 0;
#endif
        /* Map pH 0-14 → bar 0-100 */
        ph_level = (int)(cur_ph / 14.0f * 100.0f);
        if (ph_level < 0)   ph_level = 0;
        if (ph_level > 100) ph_level = 100;

        /* Map temperature 0-50 °C → bar 0-100 */
        if (temp_ok) {
            tmp_level = (int)(cur_temp / 50.0f * 100.0f);
            if (tmp_level < 0)   tmp_level = 0;
            if (tmp_level > 100) tmp_level = 100;
        }

        float threat = ((float)ph_level  / 100.0f +
                        (float)al_level  / 100.0f +
                        (float)tmp_level / 100.0f +
                        (float)trb_level / 100.0f) / 4.0f;

        BeginDrawing();
        ClearBackground((Color){12, 12, 18, 255});

        for (int gx = 0; gx < 800; gx += 40)
            DrawLine(gx, 0,   gx, 520, (Color){255, 255, 255, 8});
        for (int gy = 0; gy < 520; gy += 40)
            DrawLine(0,  gy, 800, gy,  (Color){255, 255, 255, 8});

        DrawText("ALGAE-HALT",                 50, 15, 30, WHITE);
        DrawText("WATER QUALITY MONITOR v0.1", 50, 48, 13, (Color){100, 100, 120, 255});
        DrawLine(50, 68, 580, 68, (Color){255, 255, 255, 40});

        /* Tiny serial status next to the subtitle */
        DrawText(serial_ok ? "● LIVE " COM_PORT : "○ NO SIGNAL " COM_PORT,
                 390, 52, 11, serial_ok ? GREEN : RED);

        draw_sensor_bar("pH LEVEL",    "pH",  &ph_level,  100, &ph_timer,  90);
        draw_sensor_bar("ALUMINIUM",   "ppb", &al_level,  100, &al_timer,  175);
        draw_sensor_bar("TEMPERATURE", "C",   &tmp_level, 100, &tmp_timer, 260);
        draw_sensor_bar("TURBIDITY",   "NTU", &trb_level, 100, &trb_timer, 345);

        DrawLine(50, 405, 580, 405, (Color){255, 255, 255, 40});

        const char *status = threat > 0.7f ? "STATUS: DANGER"  :
                             threat > 0.4f ? "STATUS: WARNING" :
                                             "STATUS: NOMINAL";
        Color sc = threat > 0.7f ? RED : threat > 0.4f ? YELLOW : GREEN;
        DrawText(status,                   50, 418, 18, sc);
        DrawText("LAKE MALAREN - NODE 01", 50, 442, 13, (Color){80, 80, 100, 255});
        DrawText(TextFormat("UPTIME: %.0fs", time), 50, 460, 13, (Color){80, 80, 100, 255});

        draw_threat_meter(threat);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}