/*
 * test_fsd_core.c — host unit tests for the Tesla FSD protocol core.
 *
 * Compiles fsd_logic/fsd_handler.c on the host (no Flipper SDK, no SPI) and
 * checks the bit-packing, mux dispatch, HW3/HW4 branching, checksum, and
 * signal-parsing behavior with INDEPENDENT oracles (the expected checksum is
 * recomputed here, not snapshotted from the implementation).
 *
 * Purpose: lock the current behavior before the protocol core is converged
 * into a single shared file. A frame the car silently rejects (wrong
 * checksum / wrong bit) is the exact failure mode that shipped as the v2.14
 * HW3 regression — these tests turn that into a red CI build.
 *
 * Build + run:  make -C test check
 */

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "fsd_can_ops.h"
#include "fsd_capture.h"
#include "fsd_checksum.h"
#include "fsd_handler.h"

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

static void zero(CANFRAME* f) {
    memset(f, 0, sizeof(*f));
}

// ── bit-packing primitive ───────────────────────────────────────────────────
static void test_set_bit(void) {
    CANFRAME f;
    zero(&f);
    fsd_set_bit(&f, 46, true); // 46/8=5, 46%8=6 -> 0x40
    CHECK(f.buffer[5] == 0x40, "bit46 -> buffer[5]=0x%02X exp 0x40", f.buffer[5]);
    fsd_set_bit(&f, 46, false);
    CHECK(f.buffer[5] == 0x00, "bit46 clear -> buffer[5]=0x%02X exp 0x00", f.buffer[5]);
    fsd_set_bit(&f, 60, true); // 60/8=7, 60%8=4 -> 0x10
    CHECK(f.buffer[7] == 0x10, "bit60 -> buffer[7]=0x%02X exp 0x10", f.buffer[7]);

    CANFRAME g;
    zero(&g);
    fsd_set_bit(&g, 64, true); // out of range
    fsd_set_bit(&g, -1, true);
    int all_zero = 1;
    for (int i = 0; i < 8; i++)
        if (g.buffer[i]) all_zero = 0;
    CHECK(all_zero, "out-of-range bit must be a no-op");
}

// ── mux id ────────────────────────────────────────────────────────────────────
static void test_read_mux(void) {
    CANFRAME f;
    zero(&f);
    f.buffer[0] = 0x05;
    CHECK(fsd_read_mux_id(&f) == 5, "mux 5 got %u", fsd_read_mux_id(&f));
    f.buffer[0] = 0x0F; // upper bits must be masked off
    CHECK(fsd_read_mux_id(&f) == 7, "mux mask 0x07 got %u", fsd_read_mux_id(&f));
}

// ── UI FSD-selected flag ──────────────────────────────────────────────────────
static void test_is_selected(void) {
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[4] = 0x40; // byte4 bit6
    CHECK(fsd_is_selected_in_ui(&f, false) == true, "bit6 set -> selected");
    f.buffer[4] = 0x00;
    CHECK(fsd_is_selected_in_ui(&f, false) == false, "bit6 clear -> not selected");
    CHECK(fsd_is_selected_in_ui(&f, true) == true, "force_fsd overrides UI flag");
    f.data_lenght = 4;
    CHECK(fsd_is_selected_in_ui(&f, false) == false, "dlc<5 guard -> not selected");
}

// ── HW version detection from 0x398 ───────────────────────────────────────────
static void test_detect_hw(void) {
    CANFRAME f;
    zero(&f);
    f.canId = CAN_ID_GTW_CAR_CONFIG;
    f.data_lenght = 8;
    f.buffer[0] = 0x80; // bits7:6 = 0b10 = 2
    CHECK(fsd_detect_hw_version(&f) == TeslaHW_HW3, "das_hw=2 -> HW3");
    f.buffer[0] = 0xC0; // 0b11 = 3
    CHECK(fsd_detect_hw_version(&f) == TeslaHW_HW4, "das_hw=3 -> HW4");
    f.buffer[0] = 0x00; // 0
    CHECK(fsd_detect_hw_version(&f) == TeslaHW_Legacy, "das_hw=0 -> Legacy");
    f.buffer[0] = 0x40; // 1
    CHECK(fsd_detect_hw_version(&f) == TeslaHW_Legacy, "das_hw=1 -> Legacy");
    f.canId = 0x123; // wrong id
    CHECK(fsd_detect_hw_version(&f) == TeslaHW_Unknown, "wrong id -> Unknown");
}

// ── follow distance -> speed profile ──────────────────────────────────────────
static void test_follow_distance(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 6;

    s.hw_version = TeslaHW_HW3;
    f.buffer[5] = (uint8_t)(1u << 5);
    fsd_handle_follow_distance(&s, &f);
    CHECK(s.speed_profile == 2, "HW3 fd1 -> profile2 got %d", s.speed_profile);
    f.buffer[5] = (uint8_t)(3u << 5);
    fsd_handle_follow_distance(&s, &f);
    CHECK(s.speed_profile == 0, "HW3 fd3 -> profile0 got %d", s.speed_profile);

    s.hw_version = TeslaHW_HW4;
    f.buffer[5] = (uint8_t)(5u << 5);
    fsd_handle_follow_distance(&s, &f);
    CHECK(s.speed_profile == 4, "HW4 fd5 -> profile4 got %d", s.speed_profile);
    f.buffer[5] = (uint8_t)(1u << 5);
    fsd_handle_follow_distance(&s, &f);
    CHECK(s.speed_profile == 3, "HW4 fd1 -> profile3 got %d", s.speed_profile);
}

// ── 0x3FD autopilot frame, HW4 ────────────────────────────────────────────────
static void test_autopilot_hw4(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.hw_version = TeslaHW_HW4;
    s.force_fsd = true;
    s.speed_profile = 4;

    // mux0 -> FSD activation bits 46 + 60
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 0;
    CHECK(fsd_handle_autopilot_frame(&s, &f), "HW4 mux0 reports modified");
    CHECK((f.buffer[5] & 0x40) != 0, "HW4 mux0 bit46 set");
    CHECK((f.buffer[7] & 0x10) != 0, "HW4 mux0 bit60 set");
    CHECK(s.fsd_enabled, "HW4 mux0 sets fsd_enabled");

    // mux1 -> nag bit19 cleared, bit47 set
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 1;
    f.buffer[2] = 0x08; // pre-set bit19 so we prove it is cleared
    CHECK(fsd_handle_autopilot_frame(&s, &f), "HW4 mux1 reports modified");
    CHECK((f.buffer[2] & 0x08) == 0, "HW4 mux1 bit19 cleared");
    CHECK((f.buffer[5] & 0x80) != 0, "HW4 mux1 bit47 set");

    // mux2 -> speed profile written to byte7 bits7:5
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 2;
    CHECK(fsd_handle_autopilot_frame(&s, &f), "HW4 mux2 reports modified");
    CHECK(((f.buffer[7] >> 5) & 0x07) == 4, "HW4 mux2 speed_profile=4 got %u",
          (f.buffer[7] >> 5) & 0x07);
}

// ── 0x3FD autopilot frame, HW3 ────────────────────────────────────────────────
static void test_autopilot_hw3(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.hw_version = TeslaHW_HW3;
    s.force_fsd = true;
    s.speed_profile = 2;

    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 0; // mux0
    CHECK(fsd_handle_autopilot_frame(&s, &f), "HW3 mux0 reports modified");
    CHECK((f.buffer[5] & 0x40) != 0, "HW3 mux0 bit46 set");
    CHECK(((f.buffer[6] >> 1) & 0x03) == 2, "HW3 mux0 speed_profile bits got %u",
          (f.buffer[6] >> 1) & 0x03);
}

// ── 0x399 ISA speed chime: bit + Tesla additive checksum ──────────────────────
static void test_isa_checksum(void) {
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 0x11;
    f.buffer[1] = 0x22;
    f.buffer[2] = 0x33;
    f.buffer[3] = 0x44;
    f.buffer[4] = 0x55;
    f.buffer[5] = 0x66;
    f.buffer[6] = 0x77;

    CHECK(fsd_handle_isa_speed_chime(&f), "isa reports modified");
    CHECK((f.buffer[1] & 0x20) != 0, "isa sets bit5 of byte1");

    // Independent oracle: sum(byte0..6 AFTER the bit set) + id_lo + id_hi.
    // CAN_ID_ISA_SPEED = 0x399 -> lo 0x99, hi 0x03.
    uint8_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += f.buffer[i];
    sum = (uint8_t)(sum + 0x99 + 0x03);
    CHECK(f.buffer[7] == sum, "isa checksum got 0x%02X exp 0x%02X", f.buffer[7], sum);
}

// ── 0x257 DI_speed parse ──────────────────────────────────────────────────────
static void test_di_speed(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[1] = 0x10;
    f.buffer[2] = 0x27;
    f.buffer[3] = 0x42;
    fsd_handle_di_speed(&s, &f);
    // raw = (0x27<<4)|(0x10>>4) = 0x270|1 = 625 ; 625*0.08 - 40 = 10.0
    CHECK(fabs(s.vehicle_speed_kph - 10.0f) < 0.01f, "speed got %.3f exp 10.0",
          (double)s.vehicle_speed_kph);
    CHECK(s.ui_speed == 0x42, "ui_speed got 0x%02X exp 0x42", s.ui_speed);
    CHECK(s.speed_seen, "speed_seen set after parse");
}

// ── 0x331 TLSSC restore ───────────────────────────────────────────────────────
static void test_tlssc_restore(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.tlssc_restore = true;
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;

    f.buffer[0] = 0x00;
    CHECK(fsd_handle_tlssc_restore(&s, &f), "tlssc modifies fresh byte0");
    CHECK(f.buffer[0] == 0x1B, "tlssc byte0 -> 0x1B got 0x%02X", f.buffer[0]);
    // already-restored frame: no change, returns false
    CHECK(fsd_handle_tlssc_restore(&s, &f) == false, "tlssc no-op when already 0x1B");
    // upper 2 bits preserved
    f.buffer[0] = 0xC5;
    CHECK(fsd_handle_tlssc_restore(&s, &f), "tlssc modifies 0xC5");
    CHECK(f.buffer[0] == 0xDB, "tlssc preserves top bits -> 0xDB got 0x%02X", f.buffer[0]);

    s.tlssc_restore = false;
    f.buffer[0] = 0x00;
    CHECK(fsd_handle_tlssc_restore(&s, &f) == false, "tlssc disabled -> no-op");
}

// ── 0x313 track-mode inject + additive checksum ───────────────────────────────
static void test_track_mode_crc(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.op_mode = OpMode_Service; // gated behind Service mode
    s.track_mode_state = 2;     // user toggled
    CANFRAME f;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 0xF0;
    f.buffer[1] = 0x11;
    f.buffer[2] = 0x22;

    CHECK(fsd_handle_track_mode_inject(&s, &f), "track-mode reports modified");
    CHECK((f.buffer[0] & 0x03) == 0x01, "track-mode sets request ON bit");
    // Independent oracle: byte7 = (id_lo + id_hi + sum(byte0..6)) & 0xFF, 0x313.
    uint16_t sum = (0x313 & 0xFF) + ((0x313 >> 8) & 0xFF);
    for (int i = 0; i < 7; i++)
        sum += f.buffer[i];
    CHECK(f.buffer[7] == (uint8_t)(sum & 0xFF), "track-mode checksum got 0x%02X exp 0x%02X",
          f.buffer[7], (uint8_t)(sum & 0xFF));

    // Service-mode gate: not in Service -> no-op
    s.op_mode = OpMode_Active;
    CANFRAME g;
    zero(&g);
    g.data_lenght = 8;
    CHECK(fsd_handle_track_mode_inject(&s, &g) == false, "track-mode gated outside Service");
}

// ── 0x249 SCCM_leftStalk builders + CRC ───────────────────────────────────────
static uint8_t sccm_expected_crc(const CANFRAME* f) {
    return (uint8_t)(((0x249 & 0xFF) + ((0x249 >> 8) & 0xFF) + f->buffer[1] + f->buffer[2]) & 0xFF);
}

static void test_sccm_crc(void) {
    CANFRAME f;

    fsd_build_highbeam_flash(&f, 3, true);
    CHECK(f.canId == CAN_ID_SCCM_LSTALK, "highbeam id 0x249");
    CHECK(f.data_lenght == 3, "highbeam dlc 3");
    CHECK((f.buffer[1] & 0x0F) == 3, "highbeam counter=3");
    CHECK((f.buffer[1] & 0x30) == 0x10, "highbeam PULL bit (status=1)");
    CHECK(f.buffer[0] == sccm_expected_crc(&f), "highbeam CRC got 0x%02X exp 0x%02X",
          f.buffer[0], sccm_expected_crc(&f));

    fsd_build_turn_signal(&f, 5, 3); // 3 = DOWN_1 (left)
    CHECK((f.buffer[1] & 0x0F) == 5, "turn counter=5");
    CHECK((f.buffer[2] & 0x07) == 3, "turn direction=3");
    CHECK(f.buffer[0] == sccm_expected_crc(&f), "turn CRC got 0x%02X exp 0x%02X",
          f.buffer[0], sccm_expected_crc(&f));

    fsd_build_wiper_wash(&f, 2);
    CHECK((f.buffer[1] & 0x0F) == 2, "wiper counter=2");
    CHECK((f.buffer[1] & 0xC0) == 0x40, "wiper 1ST_DETENT bit");
    CHECK(f.buffer[0] == sccm_expected_crc(&f), "wiper CRC got 0x%02X exp 0x%02X",
          f.buffer[0], sccm_expected_crc(&f));
}

// ── 0x370 nag killer: counter+1, hands-on spoof, self-consistent checksum ─────
static void test_nag_killer(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.nag_killer = true;
    s.das_hands_on_state = 0xFF; // no DAS frame seen -> conservative echo
    s.nag_demand_active = false;

    CANFRAME in;
    zero(&in);
    in.data_lenght = 8;
    in.buffer[4] = 0x00; // handsOnLevel = 0 (nag imminent)
    in.buffer[6] = 0x05; // counter = 5

    CANFRAME out;
    zero(&out);
    CHECK(fsd_handle_nag_killer(&s, &in, &out), "nag echo emitted on level 0");
    CHECK(out.canId == CAN_ID_EPAS_STATUS, "nag echo id 0x370");
    CHECK(out.data_lenght == 8, "nag echo dlc 8");
    CHECK(((out.buffer[4] >> 6) & 0x03) == 1, "nag spoofs handsOnLevel=1");
    CHECK((out.buffer[6] & 0x0F) == 6, "nag counter+1 -> 6 got %u", out.buffer[6] & 0x0F);
    // Checksum self-consistency: holds regardless of the (PRNG-driven) torque bytes.
    uint16_t sum = 0;
    for (int i = 0; i < 7; i++)
        sum += out.buffer[i];
    sum += (CAN_ID_EPAS_STATUS & 0xFF) + (CAN_ID_EPAS_STATUS >> 8);
    CHECK(out.buffer[7] == (uint8_t)(sum & 0xFF), "nag checksum self-consistent got 0x%02X exp 0x%02X",
          out.buffer[7], (uint8_t)(sum & 0xFF));

    // skip paths
    CANFRAME out2;
    zero(&out2);
    in.buffer[4] = 0x40; // handsOnLevel = 1 (hands detected)
    CHECK(fsd_handle_nag_killer(&s, &in, &out2) == false, "nag skips when hands detected");
    in.buffer[4] = 0x00;
    s.das_hands_on_state = 0; // DAS satisfied
    CHECK(fsd_handle_nag_killer(&s, &in, &out2) == false, "nag skips when DAS satisfied");
    s.das_hands_on_state = 0xFF;
    s.nag_killer = false;
    CHECK(fsd_handle_nag_killer(&s, &in, &out2) == false, "nag skips when disabled");
}

// ── shared stateless ops (china_mode path the Flipper wrapper can't reach) ────
static void test_can_ops(void) {
    uint8_t data[8] = {0};
    data[4] = 0x00; // UI flag clear
    CHECK(tesla_is_fsd_selected(data, 8, false, false) == false, "ops: not selected");
    CHECK(tesla_is_fsd_selected(data, 8, true, false) == true, "ops: force_fsd bypass");
    CHECK(tesla_is_fsd_selected(data, 8, false, true) == true, "ops: china_mode bypass");
    CHECK(tesla_is_fsd_selected(data, 4, false, false) == false, "ops: dlc<5 guard");
    data[4] = 0x40;
    CHECK(tesla_is_fsd_selected(data, 8, false, false) == true, "ops: UI bit6 selected");
    // set_bit / read_mux raw-pointer forms
    uint8_t d2[8] = {0};
    tesla_set_bit(d2, 47, true); // byte5 bit7
    CHECK(d2[5] == 0x80, "ops: set_bit 47 -> 0x80 got 0x%02X", d2[5]);
    d2[0] = 0x0E;
    CHECK(tesla_read_mux(d2) == 6, "ops: read_mux 0x0E&0x07 = 6 got %u", tesla_read_mux(d2));
}

// ── shared additive checksum kernel ───────────────────────────────────────────
static void test_additive_checksum(void) {
    uint8_t d[7] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77};
    // 0x399 -> 0x99 + 0x03 + sum(d) ; sum(d) = 0x1B8 -> +0x9C = 0x254 -> 0x54
    uint16_t s = 0x99 + 0x03;
    for (int i = 0; i < 7; i++)
        s += d[i];
    CHECK(tesla_additive_checksum(0x399, d, 7) == (uint8_t)(s & 0xFF),
          "kernel 0x399 got 0x%02X exp 0x%02X", tesla_additive_checksum(0x399, d, 7),
          (uint8_t)(s & 0xFF));
    // 2-byte SCCM-style range
    uint8_t two[2] = {0x42, 0x07};
    CHECK(tesla_additive_checksum(0x249, two, 2) ==
              (uint8_t)((0x49 + 0x02 + 0x42 + 0x07) & 0xFF),
          "kernel 0x249 2-byte");
    // zero-length: just the folded id bytes
    CHECK(tesla_additive_checksum(0x370, d, 0) == (uint8_t)(0x70 + 0x03),
          "kernel len=0 -> id fold only");
}

// ── shared candump-ASCII formatter (capture-first / cracker input) ────────────
static void test_candump_format(void) {
    char buf[48];
    uint8_t d[8] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0xAA};
    int n = tesla_format_candump_line(buf, sizeof(buf), 1500, "can0", 0x485, d, 8);
    CHECK(strcmp(buf, "(1.500000) can0 485#00112233445566AA\n") == 0,
          "candump 8-byte: [%s]", buf);
    CHECK(n == (int)strlen(buf), "candump returns byte count (%d vs %zu)", n, strlen(buf));

    // 11-bit ID is zero-padded to 3 hex; short DLC; elapsed seconds carry.
    uint8_t two[2] = {0xDE, 0xAD};
    tesla_format_candump_line(buf, sizeof(buf), 2007, "can0", 0x7, two, 2);
    CHECK(strcmp(buf, "(2.007000) can0 007#DEAD\n") == 0, "candump short: [%s]", buf);

    // DLC clamped to 8 even if a bogus larger value is passed.
    tesla_format_candump_line(buf, sizeof(buf), 0, "can0", 0x3FD, d, 200);
    CHECK(strcmp(buf, "(0.000000) can0 3FD#00112233445566AA\n") == 0, "candump dlc clamp: [%s]", buf);
}

// ── 0x318 GTW_carState OTA detection (gates TX) ───────────────────────────────
static void test_gtw_car_state(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 7;
    f.buffer[6] = 2; // installing
    fsd_handle_gtw_car_state(&s, &f);
    CHECK(s.tesla_ota_in_progress, "OTA installing(2) -> in_progress");
    f.buffer[6] = 1; // available — must NOT pause TX (issue #19 false positive)
    fsd_handle_gtw_car_state(&s, &f);
    CHECK(!s.tesla_ota_in_progress, "OTA available(1) -> not in_progress");
    f.buffer[6] = 0;
    fsd_handle_gtw_car_state(&s, &f);
    CHECK(!s.tesla_ota_in_progress, "OTA none(0) -> not in_progress");
}

// ── 0x045 Legacy stalk + 0x3EE Legacy autopilot ───────────────────────────────
static void test_legacy(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 2;
    f.buffer[1] = (uint8_t)(0u << 5);
    fsd_handle_legacy_stalk(&s, &f);
    CHECK(s.speed_profile == 2, "legacy stalk pos0 -> 2 got %d", s.speed_profile);
    f.buffer[1] = (uint8_t)(2u << 5);
    fsd_handle_legacy_stalk(&s, &f);
    CHECK(s.speed_profile == 1, "legacy stalk pos2 -> 1 got %d", s.speed_profile);
    f.buffer[1] = (uint8_t)(4u << 5);
    fsd_handle_legacy_stalk(&s, &f);
    CHECK(s.speed_profile == 0, "legacy stalk pos4 -> 0 got %d", s.speed_profile);

    s.force_fsd = true;
    s.speed_profile = 2;
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 0; // mux0
    CHECK(fsd_handle_legacy_autopilot(&s, &f), "legacy AP mux0 modified");
    CHECK((f.buffer[5] & 0x40) != 0, "legacy AP mux0 bit46");
    CHECK(((f.buffer[6] >> 1) & 0x03) == 2, "legacy AP mux0 speed profile");
    zero(&f);
    f.data_lenght = 8;
    f.buffer[0] = 1; // mux1
    f.buffer[2] = 0x08; // bit19 preset
    CHECK(fsd_handle_legacy_autopilot(&s, &f), "legacy AP mux1 modified");
    CHECK((f.buffer[2] & 0x08) == 0, "legacy AP mux1 bit19 cleared");
}

// ── 0x145 ESP_status brake ────────────────────────────────────────────────────
static void test_esp_status(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 4;
    f.buffer[3] = 0x20; // bits[6:5] != 0
    fsd_handle_esp_status(&s, &f);
    CHECK(s.driver_brake_applied, "esp brake applied");
    f.buffer[3] = 0x00;
    fsd_handle_esp_status(&s, &f);
    CHECK(!s.driver_brake_applied, "esp no brake");
}

// ── DAS_status parsers (nag-killer gating source) ─────────────────────────────
static void test_das_status(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    CANFRAME f;
    zero(&f);
    f.data_lenght = 6;
    f.buffer[0] = 0x03;          // HW3 ap_state = 3 (low nibble)
    f.buffer[5] = (uint8_t)(0x05 << 2); // hands_on = 5 (bits[5:2])
    fsd_handle_das_status_hw3(&s, &f);
    CHECK(s.das_ap_state == 3, "hw3 ap_state got %u", s.das_ap_state);
    CHECK(s.das_hands_on_state == 5, "hw3 hands_on got %u", s.das_hands_on_state);
    CHECK(s.das_seen, "hw3 das_seen");

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 7;
    f.buffer[1] = (uint8_t)(0x02 << 4); // ap_state = 2 (bits[7:4])
    f.buffer[5] = (uint8_t)(0x03 << 2); // hands_on = 3
    f.buffer[4] = 0x02;                 // side_coll_warn = 2 (bits[1:0])
    f.buffer[2] = 0xC0 | 0x05;          // fcw = 3 (bits[7:6]), vision = 5 (bits[4:0])
    fsd_handle_das_status_hw4(&s, &f);
    CHECK(s.das_ap_state == 2, "hw4 ap_state got %u", s.das_ap_state);
    CHECK(s.das_hands_on_state == 3, "hw4 hands_on got %u", s.das_hands_on_state);
    CHECK(s.das_side_coll_warn == 2, "hw4 side_coll got %u", s.das_side_coll_warn);
    CHECK(s.das_fcw == 3, "hw4 fcw got %u", s.das_fcw);
    CHECK(s.das_vision_speed_lim == 5, "hw4 vision got %u", s.das_vision_speed_lim);
    CHECK(s.das_seen, "hw4 das_seen");
}

// ── 0x7FF tier parse + active override ────────────────────────────────────────
static void test_gtw_tier(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    s.gtw_autopilot_tier = -1;
    CANFRAME f;
    zero(&f);
    f.data_lenght = 6;
    f.buffer[0] = 2;                    // mux 2
    f.buffer[5] = (uint8_t)(0x03 << 2); // tier = 3
    fsd_handle_gtw_autopilot_tier(&s, &f);
    CHECK(s.gtw_autopilot_tier == 3, "gtw tier parse got %d", s.gtw_autopilot_tier);
    s.gtw_autopilot_tier = -1;
    f.buffer[0] = 1; // wrong mux ignored
    fsd_handle_gtw_autopilot_tier(&s, &f);
    CHECK(s.gtw_autopilot_tier == -1, "gtw tier mux!=2 ignored");

    s.gtw_tier_override = true;
    zero(&f);
    f.data_lenght = 6;
    f.buffer[0] = 2;
    f.buffer[5] = 0x00;
    CHECK(fsd_handle_gtw_tier_override(&s, &f), "tier override modifies");
    CHECK(((f.buffer[5] >> 2) & 0x07) == 3, "tier override -> 3 got %u", (f.buffer[5] >> 2) & 0x07);
    s.gtw_tier_override = false;
    f.buffer[5] = 0x00;
    CHECK(fsd_handle_gtw_tier_override(&s, &f) == false, "tier override disabled -> noop");
}

// ── 0x3F8 driver-assist override bit map ──────────────────────────────────────
static void test_driver_assist(void) {
    FSDState s;
    CANFRAME f;

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 8;
    s.assist_dev_mode = true;
    CHECK(fsd_handle_driver_assist_override(&s, &f), "assist dev modifies");
    CHECK((f.buffer[0] & 0x20) != 0, "assist dev bit5");

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 8;
    s.assist_nav_enable = true;
    fsd_handle_driver_assist_override(&s, &f);
    CHECK((f.buffer[1] & 0x20) != 0, "assist nav bit13");
    CHECK((f.buffer[6] & 0x01) != 0, "assist nav bit48");
    CHECK((f.buffer[6] & 0x02) != 0, "assist nav bit49");

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 8;
    s.assist_lhd_override = true;
    fsd_handle_driver_assist_override(&s, &f);
    CHECK((f.buffer[5] & 0x01) != 0, "assist lhd bit40 set");
    CHECK((f.buffer[5] & 0x02) == 0, "assist lhd bit41 clear");

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 8;
    f.buffer[5] = 0x08; // bit43 preset
    s.assist_telemetry_off = true;
    fsd_handle_driver_assist_override(&s, &f);
    CHECK((f.buffer[5] & 0x08) == 0, "assist telemetry bit43 cleared");

    memset(&s, 0, sizeof(s));
    zero(&f);
    f.data_lenght = 8;
    CHECK(fsd_handle_driver_assist_override(&s, &f) == false, "assist no flags -> noop");
}

// ── 0x7FF GTW Config Replay: learn -> arm -> replay ───────────────────────────
static void test_gtw_shield(void) {
    FSDState s;
    memset(&s, 0, sizeof(s));
    // Learning: feed all 8 mux frames; none transmit, arms after the 8th.
    for(uint8_t m = 0; m < 8; m++) {
        CANFRAME f;
        zero(&f);
        f.data_lenght = 8;
        f.buffer[0] = m;       // mux
        f.buffer[3] = 0xAA;    // "healthy" payload
        CHECK(fsd_handle_gtw_shield(&s, &f) == false, "shield learning -> false (mux %u)", m);
    }
    CHECK(s.gtw_shield_armed, "shield armed after 8 mux snapshots");

    // Armed, unchanged frame -> no replay.
    CANFRAME ok;
    zero(&ok);
    ok.data_lenght = 8;
    ok.buffer[0] = 0;
    ok.buffer[3] = 0xAA;
    CHECK(fsd_handle_gtw_shield(&s, &ok) == false, "shield unchanged -> false");

    // Armed, tampered frame -> replay healthy snapshot.
    CANFRAME bad;
    zero(&bad);
    bad.data_lenght = 8;
    bad.buffer[0] = 0;
    bad.buffer[3] = 0xBB; // gateway changed it
    CHECK(fsd_handle_gtw_shield(&s, &bad), "shield tampered -> true (replay)");
    CHECK(bad.buffer[3] == 0xAA, "shield restored byte3 to 0xAA got 0x%02X", bad.buffer[3]);
    CHECK(s.gtw_shield_blocks == 1, "shield block counted");
}

// ── state init ────────────────────────────────────────────────────────────────
static void test_state_init(void) {
    FSDState s;
    fsd_state_init(&s, TeslaHW_HW4);
    CHECK(s.hw_version == TeslaHW_HW4, "init applies HW4");
}

int main(void) {
    printf("test_fsd_core: Tesla FSD protocol core host tests\n");
    test_set_bit();
    test_read_mux();
    test_is_selected();
    test_detect_hw();
    test_follow_distance();
    test_autopilot_hw4();
    test_autopilot_hw3();
    test_isa_checksum();
    test_di_speed();
    test_tlssc_restore();
    test_track_mode_crc();
    test_sccm_crc();
    test_nag_killer();
    test_can_ops();
    test_additive_checksum();
    test_candump_format();
    test_gtw_car_state();
    test_legacy();
    test_esp_status();
    test_das_status();
    test_gtw_tier();
    test_driver_assist();
    test_gtw_shield();
    test_state_init();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
