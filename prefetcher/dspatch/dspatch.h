#ifndef DSPATCH_H
#define DSPATCH_H

#include <stdint.h>
#include <bitset>

// Configuration parameters
constexpr uint32_t REGION_SIZE_BYTES = 4096; // 4KB
constexpr uint32_t CACHE_LINE_SIZE_BYTES = 64; // 64B
constexpr uint32_t LINES_PER_REGION = REGION_SIZE_BYTES / CACHE_LINE_SIZE_BYTES; // 64 lines per region
constexpr uint32_t PB_SIZE = 64; // Number of pages tracked in Page Buffer
constexpr uint32_t SPT_SIZE = 256; // Number of entries in Spatial Pattern Table

class SaturatingCounter {
private:
    uint32_t value;
    uint32_t max_value;
public:
    SaturatingCounter(uint32_t bits) : value(0), max_value((1<<bits) -1) {}
    void increment() {
        if (value < max_value) value++;
    }
    void decrement() {
        if (value > 0) value--;
    }
    void reset() {
        value = 0;
    }
    uint32_t get() const {
        return value;
    }
    bool is_saturated() const {
        return value == max_value;
    }
};

class BitMath {
public:
    std::bitset<LINES_PER_REGION/2> compress(const std::bitset<LINES_PER_REGION> &bitmap) {
        std::bitset<LINES_PER_REGION/2> compressed;
        for (size_t i = 0; i < LINES_PER_REGION/2; ++i) {
            compressed[i] = bitmap[2*i] | bitmap[2*i + 1];
        }
        return compressed;
    }

    std::bitset<LINES_PER_REGION> decompress(const std::bitset<LINES_PER_REGION/2> &compressed) {
        std::bitset<LINES_PER_REGION> bitmap;
        for (size_t i = 0; i < LINES_PER_REGION/2; ++i) {
            bitmap[2*i] = compressed[i];
            bitmap[2*i + 1] = compressed[i];
        }
        return bitmap;
    }

    // Circular shift left
    std::bitset<LINES_PER_REGION/2> circular_shift_left(const std::bitset<LINES_PER_REGION/2> &bitmap, size_t shift) {
        if (shift == 0) return bitmap;
        return (bitmap << shift) | (bitmap >> (LINES_PER_REGION/2 - shift));
    }

    // Circular shift right
    std::bitset<LINES_PER_REGION/2> circular_shift_right(const std::bitset<LINES_PER_REGION/2> &bitmap, size_t shift) {
        if (shift == 0) return bitmap;
        return (bitmap >> shift) | (bitmap << (LINES_PER_REGION/2 - shift));
    }

    static uint32_t popcount(const std::bitset<LINES_PER_REGION/2> &bitmap) {
        return bitmap.count();
    }
};

// Page Buffer -> Track spatial footprint
// Tracks recent 4KB physical pages and records which cache line inside
// the page were touched - stored as bit pattern
// 1st access to each 2KB half-page acts as a  trigger

class PB_Entry {
public:
    uint64_t page_addr;
    uint64_t trigger_pc;
    uint32_t trigger_offset; // offset within the page that triggered the entry

    std::bitset<LINES_PER_REGION> access_bitmap; // bit pattern of accessed cache lines within the page

    PB_Entry(uint64_t page_addr, uint64_t pc, uint32_t offset)
        : page_addr(page_addr), trigger_pc(pc), trigger_offset(offset) {
            access_bitmap.reset();
            access_bitmap.set(offset);
        }
};

// SPT -> store learned patterns
// Patterns indexed by trigger PC
// Tracks pattern quality
// 256 entry direct-mapped table -> see config up top and change as needed

class SPT_Entry {
public:
    // Granualioty is 128B -> 1bit  == 2 cache lines
    std::bitset<LINES_PER_REGION/2> bmp_covP;
    std::bitset<LINES_PER_REGION/2> bmp_accP;

    // 2 bit Counters
    SaturatingCounter measure_covP; // coverage predictor
    SaturatingCounter measure_accP; // accuracy predictor
    SaturatingCounter or_count;

    SPT_Entry() : measure_covP(2), measure_accP(2), or_count(2) {
        bmp_covP.reset();
        bmp_accP.reset();
    }
};


class DSPatchCore {

}

#endif // DSPATCH_H