/* Cross-check harness: transcribes the exact ExpressLRS algorithms (LCG,
 * FHSS sequence build, Crc2Byte) from ~/ExpressLRS source and prints outputs
 * for fixed inputs. compare_xcheck.py runs the Python port on the same inputs
 * and asserts byte-identical results -- decisive proof the port is exact.
 *
 * Transcribed from:
 *   lib/FHSS/random.cpp   rng()
 *   lib/FHSS/FHSS.cpp:134-175  FHSSrandomiseFHSSsequenceBuild
 *   lib/CRC/crc.cpp       Crc2Byte::init / calc
 */
#include <stdio.h>
#include <stdint.h>

/* ---- random.cpp ---- */
static uint32_t g_seed = 0;
static uint16_t rng(void) {
    const uint32_t m = 2147483648u, a = 214013u, c = 2531011u;
    g_seed = (a * g_seed + c) % m;
    return g_seed >> 16;
}
static void rngSeed(uint32_t s) { g_seed = s; }
static uint8_t rngN(uint8_t max) { return rng() % max; }

/* ---- FHSS.cpp sequence build (ISM2G4: freqCount=80, sync=40, count=240) ---- */
#define SEQLEN 240
static void build(uint32_t seed, uint32_t freqCount, uint8_t sync, uint8_t *seq) {
    rngSeed(seed);
    for (uint16_t i = 0; i < SEQLEN; i++) {
        if (i % freqCount == 0)          seq[i] = sync;
        else if (i % freqCount == sync)  seq[i] = 0;
        else                             seq[i] = i % freqCount;
    }
    for (uint16_t i = 0; i < SEQLEN; i++) {
        if (i % freqCount != 0) {
            uint8_t offset = (i / freqCount) * freqCount;
            uint8_t rand = rngN(freqCount - 1) + 1;
            uint8_t tmp = seq[i];
            seq[i] = seq[offset + rand];
            seq[offset + rand] = tmp;
        }
    }
}

/* ---- crc.cpp Crc2Byte ---- */
static uint16_t crctab[256];
static uint8_t  cb_bits;
static uint16_t cb_mask;
static void crc_init(uint8_t bits, uint16_t poly) {
    cb_bits = bits; cb_mask = (1 << bits) - 1;
    uint16_t highbit = 1 << (bits - 1);
    for (uint16_t i = 0; i < 256; i++) {
        uint16_t crc = i << (bits - 8);
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc << 1) ^ ((crc & highbit) ? poly : 0);
        crctab[i] = crc;
    }
}
static uint16_t crc_calc(const uint8_t *data, uint8_t len, uint16_t crc) {
    while (len--)
        crc = (crc << 8) ^ crctab[((crc >> (cb_bits - 8)) ^ (uint16_t)*data++) & 0x00FF];
    return crc & cb_mask;
}

int main(void) {
    uint8_t seq[SEQLEN];
    uint32_t seeds[] = {0u, 1u, 0xDEADBEEFu, 0xA5663E7Cu, 12345u};
    for (int s = 0; s < 5; s++) {
        build(seeds[s], 80, 40, seq);
        printf("SEQ %u", seeds[s]);
        for (int i = 0; i < SEQLEN; i++) printf(" %d", seq[i]);
        printf("\n");
    }
    /* CRC14 over a 7-byte body, then CRC16 over 11-byte body */
    crc_init(14, 0x2E57);
    uint8_t body7[7] = {0x01, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC};
    printf("CRC14 %u\n", crc_calc(body7, 7, 0x1234 & 0x3FFF));
    crc_init(16, 0x3D65);
    uint8_t body11[11] = {0x01, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    printf("CRC16 %u\n", crc_calc(body11, 11, 0xABCD));
    return 0;
}
