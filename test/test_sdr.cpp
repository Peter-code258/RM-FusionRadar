// test_sdr.cpp - Unit tests for SDR decoder
#include "sdr_decoder.h"
#include "kalman_interpolator.h"
#include <cassert>
#include <iostream>
#include <cstring>

using namespace rm_radar;
using namespace rm_radar::sdr;

static int passed = 0, failed = 0;
#define TEST(name) std::cout << "[TEST] " << name << " ... "
#define PASS() do { std::cout << "PASS" << std::endl; passed++; } while(0)
#define FAIL(m) do { std::cout << "FAIL: " << m << std::endl; failed++; } while(0)

static SdrFrame makeValidFrame(uint8_t id, float x, float y) {
    SdrFrame f;
    std::memset(&f, 0, sizeof(f));
    f.sync = SYNC_WORD;
    f.length = SDR_PAYLOAD_SIZE;
    f.robot_id = id;
    f.x = x; f.y = y;
    f.hp = 1000; f.ammo = 40; f.status = 1;
    f.crc16 = frameCrc(f);
    return f;
}

void test_crc16() {
    TEST("CRC-16 verification");
    SdrFrame f = makeValidFrame(1, 1.0f, 2.0f);
    uint16_t calc = frameCrc(f);
    if (calc == f.crc16) PASS(); else FAIL("CRC mismatch");
}

void test_crc_rejects_bad_frame() {
    TEST("CRC rejects corrupted frame");
    SdrDecoder dec;
    dec.init();
    SdrFrame f = makeValidFrame(1, 1.0f, 2.0f);
    f.crc16 ^= 0xFFFF; // corrupt CRC
    SdrTarget t;
    bool ok = dec.decodeFrame(f, t);
    if (!ok) PASS(); else FAIL("corrupt frame was accepted");
}

void test_frame_sync_and_decode() {
    TEST("frame sync search + decode from byte stream");
    SdrDecoder dec;
    dec.init();

    SdrFrame f = makeValidFrame(3, 4.5f, 6.7f);
    // Prepend some noise bytes
    std::vector<uint8_t> stream = {0x00, 0xFF, 0x12, 0x34};
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&f);
    stream.insert(stream.end(), p, p + sizeof(SdrFrame));

    auto targets = dec.feed(stream.data(), stream.size());
    if (targets.size() == 1 && targets[0].robot_id == 3 &&
        std::abs(targets[0].x - 4.5f) < 0.01f) {
        PASS();
    } else {
        FAIL("decoded target mismatch");
    }
}

void test_multiple_frames_in_stream() {
    TEST("decode multiple frames in one stream");
    SdrDecoder dec;
    dec.init();

    std::vector<uint8_t> stream;
    for (int i = 0; i < 3; ++i) {
        SdrFrame f = makeValidFrame(static_cast<uint8_t>(i + 1), i * 1.0f, i * 2.0f);
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&f);
        stream.insert(stream.end(), p, p + sizeof(SdrFrame));
    }
    auto targets = dec.feed(stream.data(), stream.size());
    if (targets.size() == 3) PASS(); else FAIL("expected 3 targets, got " + std::to_string(targets.size()));
}

void test_inverted_sync() {
    TEST("accept inverted sync word 0x55AA");
    SdrDecoder dec;
    dec.init();
    SdrFrame f = makeValidFrame(1, 1.0f, 1.0f);
    f.sync = SYNC_WORD_INV;
    f.crc16 = frameCrc(f); // recompute (sync not part of payload CRC)
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&f);
    auto targets = dec.feed(p, sizeof(SdrFrame));
    if (targets.size() == 1) PASS(); else FAIL("inverted sync not accepted");
}

void test_kalman_interpolation() {
    TEST("Kalman interpolator smooths trajectory");
    KalmanInterpolator ki(10, 30);
    SdrTarget t;
    t.robot_id = 1;
    t.x = 0.0f; t.y = 0.0f;
    ki.update(t);
    t.x = 1.0f; t.y = 0.0f;
    ki.update(t);
    auto interp = ki.interpolate();
    if (interp.size() == 1 && interp[0].x > 0.0f && interp[0].x < 1.5f) {
        PASS();
    } else {
        FAIL("interpolation out of expected range");
    }
}

void test_big_endian_swap() {
    TEST("byte order swap functions");
    uint16_t a = 0x1234;
    if (bswap16(bswap16(a)) == a) PASS(); else FAIL("bswap16 roundtrip failed");
}

int main() {
    std::cout << "=== SDR Decoder Unit Tests ===" << std::endl;
    test_crc16();
    test_crc_rejects_bad_frame();
    test_frame_sync_and_decode();
    test_multiple_frames_in_stream();
    test_inverted_sync();
    test_kalman_interpolation();
    test_big_endian_swap();
    std::cout << "\n=== " << passed << " passed, " << failed << " failed ===" << std::endl;
    return failed == 0 ? 0 : 1;
}
