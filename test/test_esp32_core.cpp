/*
 * test_esp32_core.cpp — host unit tests for the ESP32 firmware handlers.
 *
 * Compiles esp32/.firmware/fsd_handler.cpp with the host C++ compiler (it has
 * no Arduino dependencies), so the ESP32 copies of the protocol handlers get
 * the same red-CI coverage as the Flipper core in test_fsd_core.c. The two
 * builds disagreed on 0x318 OTA detection (#183) because nothing here ever
 * compiled the ESP32 side.
 *
 * Build + run:  make -C test check
 */

#include <stdio.h>
#include <string.h>

#include "fsd_handler.h"  // esp32/.firmware/fsd_handler.h (first on the include path)
#include "fsd_ota.h"      // shared reference: fsd_ota_update()

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (cond) {                                                             \
            g_pass++;                                                           \
        } else {                                                                \
            g_fail++;                                                           \
            printf("  FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                                \
            printf("\n");                                                       \
        }                                                                       \
    } while (0)

// Real 0x318 byte6, Model S Palladium 2022, consecutive received frames
// (decimated): a +2 rolling counter, always odd, not an update flag (#183).
static const uint8_t k_palladium_318_b6[] = {
    0x47, 0x4B, 0x4D, 0x53, 0x45, 0x4F, 0x5D, 0x4B, 0x5F, 0x49, 0x53, 0x5B, 0x43, 0x4B, 0x5B,
    0x43, 0x5F, 0x53, 0x4F, 0x4D, 0x57, 0x41, 0x5F, 0x5B, 0x4D, 0x5D, 0x49, 0x53, 0x4F,
};

static uint32_t xorshift32(uint32_t* x) {
    *x ^= *x << 13;
    *x ^= *x >> 17;
    *x ^= *x << 5;
    return *x;
}

static void esp32_feed(FSDState* s, uint8_t b6) {
    CanFrame f;
    memset(&f, 0, sizeof(f));
    f.id = CAN_ID_GTW_CAR_STATE;
    f.dlc = 8;
    f.data[6] = b6;
    fsd_handle_gtw_car_state(s, &f);
}

// Feed one byte6 sequence to the ESP32 handler and to the shared fsd_ota_update()
// the Flipper handler runs; both must agree after every frame. Returns how many
// times the ESP32 side latched.
static int parity_run(const char* name, const uint8_t* b6, int n) {
    FSDState esp, ref;
    memset(&esp, 0, sizeof(esp));
    memset(&ref, 0, sizeof(ref));
    int diverged_at = -1;
    int latches = 0;
    for (int i = 0; i < n; i++) {
        bool was = esp.tesla_ota_in_progress;
        esp32_feed(&esp, b6[i]);
        fsd_ota_update(&ref, b6[i]);
        if (diverged_at < 0 && (esp.tesla_ota_in_progress != ref.tesla_ota_in_progress ||
                                esp.ota_raw_state != ref.ota_raw_state ||
                                esp.ota_assert_count != ref.ota_assert_count ||
                                esp.ota_clear_count != ref.ota_clear_count))
            diverged_at = i;
        if (!was && esp.tesla_ota_in_progress) latches++;
    }
    CHECK(diverged_at < 0, "%s: ESP32 diverges from fsd_ota_update at frame %d", name,
          diverged_at);
    return latches;
}

// ── 0x318 GTW_carState on the ESP32 handler ───────────────────────────────────
static void test_gtw_car_state(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));

    CanFrame f;
    memset(&f, 0, sizeof(f));
    f.dlc = 6; // short frame: ignored
    f.data[6] = 0x42;
    fsd_handle_gtw_car_state(&s, &f);
    CHECK(!s.ota_last_valid && s.ota_clear_count == 0, "ESP32 OTA dlc<7 frame ignored");

    // Stable raw 1 used to latch the ESP32 (old OTA_IN_PROGRESS_RAW_VALUE); it must not.
    for (int i = 0; i < 10; i++)
        esp32_feed(&s, 0x41);
    CHECK(!s.tesla_ota_in_progress, "ESP32 OTA stable raw 1 never latches");

    // Stable raw-2 flag: latches on the 3rd repeat (frame 4), not the 2nd.
    memset(&s, 0, sizeof(s));
    for (int i = 0; i < 3; i++)
        esp32_feed(&s, 0x42);
    CHECK(!s.tesla_ota_in_progress, "ESP32 OTA 2 repeats -> not yet");
    esp32_feed(&s, 0x42);
    CHECK(s.tesla_ota_in_progress, "ESP32 OTA 3rd repeat -> latched");
    CHECK(s.ota_raw_state == 2, "ESP32 OTA raw_state 2 got %u", s.ota_raw_state);

    // Released by exactly 6 non-asserting frames.
    for (int i = 0; i < 5; i++)
        esp32_feed(&s, 0x41);
    CHECK(s.tesla_ota_in_progress, "ESP32 OTA 5 clear frames -> still latched");
    esp32_feed(&s, 0x41);
    CHECK(!s.tesla_ota_in_progress, "ESP32 OTA 6th clear frame -> released");
}

// ── ESP32 vs shared reference, frame by frame ─────────────────────────────────
static void test_ota_parity(void) {
    int run = 0, max_run = 0;
    for (size_t i = 0; i < sizeof(k_palladium_318_b6); i++) {
        run = ((k_palladium_318_b6[i] & 0x03u) == 1u) ? run + 1 : 0;
        if (run > max_run) max_run = run;
    }
    CHECK(max_run >= 3, "Palladium fixture raw-1 run %d (old ESP32 3-frame trigger)", max_run);
    CHECK(parity_run("Palladium", k_palladium_318_b6, (int)sizeof(k_palladium_318_b6)) == 0,
          "ESP32 OTA Palladium capture never latches");

    FSDState s;
    memset(&s, 0, sizeof(s));
    s.op_mode = OpMode_Active;
    bool tx_ok = true;
    for (size_t i = 0; i < sizeof(k_palladium_318_b6); i++) {
        esp32_feed(&s, k_palladium_318_b6[i]);
        if (!fsd_can_transmit(&s)) tx_ok = false;
    }
    CHECK(tx_ok, "ESP32 TX never paused by the Palladium counter");

    char name[48];
    uint8_t seq[512];

    // +2 counter 0x21..0x3F, keeping every k-th frame (2: raw pinned at 1,
    // 16: aliased to a constant 0x21).
    static const int keep_every[] = {1, 2, 3, 16};
    for (size_t k = 0; k < sizeof(keep_every) / sizeof(keep_every[0]); k++) {
        for (int i = 0; i < 128; i++)
            seq[i] = (uint8_t)(0x21 + 2 * ((i * keep_every[k]) % 16));
        snprintf(name, sizeof(name), "+2 counter keep 1/%d", keep_every[k]);
        CHECK(parity_run(name, seq, 128) == 0, "ESP32 OTA %s never latches", name);
    }

    // +2 counter under pseudo-random RX drops (fixed seed).
    for (int keep_q = 1; keep_q <= 3; keep_q++) {
        uint32_t rng = 0x31800u + (uint32_t)keep_q;
        int n = 0;
        for (int i = 0; n < 256; i++)
            if ((int)(xorshift32(&rng) & 3u) < keep_q) seq[n++] = (uint8_t)(0x21 + 2 * (i % 16));
        snprintf(name, sizeof(name), "+2 counter random drops keep %d/4", keep_q);
        CHECK(parity_run(name, seq, n) == 0, "ESP32 OTA %s never latches", name);
    }

    // +1 counter kept every 4th frame: phase 2 is a constant raw 2.
    for (int phase = 0; phase < 4; phase++) {
        for (int i = 0; i < 128; i++)
            seq[i] = (uint8_t)(phase + 4 * i);
        snprintf(name, sizeof(name), "+1 counter every 4th phase %d", phase);
        CHECK(parity_run(name, seq, 128) == 0, "ESP32 OTA %s never latches", name);
    }

    // Flag latch, release, then a changed raw-2 byte restarting the count.
    static const uint8_t flag[] = {
        0x42, 0x42, 0x42, 0x42, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41,
        0x42, 0x42, 0x42, 0x46, 0x46, 0x46, 0x46, 0x02, 0x06, 0x0A,
    };
    CHECK(parity_run("flag", flag, (int)sizeof(flag)) == 2,
          "ESP32 OTA flag sequence latches twice");

    // Random byte6 runs (1..8 frames, biased to raw 2): every transition type.
    uint32_t rng = 0x318u;
    int n = 0;
    while (n < (int)sizeof(seq)) {
        uint8_t v = (uint8_t)xorshift32(&rng);
        if (xorshift32(&rng) & 1u) v = (uint8_t)((v & 0xFCu) | 0x02u);
        int len = 1 + (int)(xorshift32(&rng) % 8u);
        for (int j = 0; j < len && n < (int)sizeof(seq); j++)
            seq[n++] = v;
    }
    CHECK(parity_run("random runs", seq, n) >= 2, "ESP32 OTA random runs latch and release");
}

// ── TX gate: OTA latch vs ignore_ota vs listen-only ───────────────────────────
static void test_ota_tx_gate(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.op_mode = OpMode_Active;
    CHECK(fsd_can_transmit(&s), "Active, no OTA -> TX allowed");
    for (int i = 0; i < 4; i++)
        esp32_feed(&s, 0x42);
    CHECK(s.tesla_ota_in_progress, "stable raw-2 flag latched");
    CHECK(!fsd_can_transmit(&s), "OTA latched -> TX blocked");
    s.ignore_ota = true;
    CHECK(fsd_can_transmit(&s), "OTA latched + ignore_ota -> TX allowed");
    s.op_mode = OpMode_ListenOnly;
    CHECK(!fsd_can_transmit(&s), "listen-only blocks TX even with ignore_ota");
    s.ignore_ota = false;
    for (int i = 0; i < 6; i++)
        esp32_feed(&s, 0x41);
    CHECK(!s.tesla_ota_in_progress, "OTA released");
    CHECK(!fsd_can_transmit(&s), "listen-only blocks TX with no OTA");
    s.op_mode = OpMode_Active;
    CHECK(fsd_can_transmit(&s), "Active after release -> TX allowed");
}

// ── state init ────────────────────────────────────────────────────────────────
static void test_state_init(void) {
    FSDState s;
    memset(&s, 0xA5, sizeof(s)); // init must not rely on a zeroed struct
    fsd_state_init(&s, TeslaHW_Unknown);
    CHECK(!s.tesla_ota_in_progress && !s.ota_last_valid && s.ota_last_byte6 == 0 &&
              s.ota_assert_count == 0 && s.ota_clear_count == 0,
          "init clears OTA detection state");
    CHECK(s.op_mode == OpMode_ListenOnly && !fsd_can_transmit(&s), "init: listen-only, TX blocked");
}

int main() {
    printf("test_esp32_core: ESP32 firmware handler host tests\n");
    test_gtw_car_state();
    test_ota_parity();
    test_ota_tx_gate();
    test_state_init();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
