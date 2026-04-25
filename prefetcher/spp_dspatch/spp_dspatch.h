#ifndef SPP_DSPATCH_H
#define SPP_DSPATCH_H

#include <bitset>
#include <cstdint>
#include <deque>
#include <unordered_set>
#include <vector>
#include <algorithm>

#include "cache.h"
#include "modules.h"
#include "msl/lru_table.h"

// ============================================================
// DSPatch Configuration
// ============================================================
constexpr uint32_t REGION_SIZE_BYTES   = 4096;
constexpr uint32_t CACHE_LINE_SIZE_BYTES = 64;
constexpr uint32_t LINES_PER_REGION    = REGION_SIZE_BYTES / CACHE_LINE_SIZE_BYTES;
constexpr uint32_t PB_SIZE             = 64;
constexpr uint32_t SPT_SIZE            = 256;

// ============================================================
// DSPatch Utilities
// ============================================================
class DSPatchCounter {
  uint32_t value, max_value;
public:
  DSPatchCounter(uint32_t bits) : value(0), max_value((1 << bits) - 1) {}
  void increment() { if (value < max_value) value++; }
  void decrement() { if (value > 0) value--; }
  void reset()     { value = 0; }
  uint32_t get()   const { return value; }
  bool is_saturated() const { return value == max_value; }
};

class DSPatchBitmapOps {
public:
  static std::bitset<LINES_PER_REGION/2> compress(const std::bitset<LINES_PER_REGION>& bmp) {
    std::bitset<LINES_PER_REGION/2> c;
    for (size_t i = 0; i < LINES_PER_REGION/2; ++i)
      c[i] = bmp[2*i] | bmp[2*i+1];
    return c;
  }
  static std::bitset<LINES_PER_REGION> decompress(const std::bitset<LINES_PER_REGION/2>& c) {
    std::bitset<LINES_PER_REGION> bmp;
    for (size_t i = 0; i < LINES_PER_REGION/2; ++i)
      bmp[2*i] = bmp[2*i+1] = c[i];
    return bmp;
  }
  static std::bitset<LINES_PER_REGION> circular_shift_left(const std::bitset<LINES_PER_REGION>& bmp, uint32_t shift) {
    if (!shift) return bmp;
    return (bmp << shift) | (bmp >> (LINES_PER_REGION - shift));
  }
  static std::bitset<LINES_PER_REGION> circular_shift_right(const std::bitset<LINES_PER_REGION>& bmp, uint32_t shift) {
    if (!shift) return bmp;
    return (bmp >> shift) | (bmp << (LINES_PER_REGION - shift));
  }
  static uint32_t popcount(const std::bitset<LINES_PER_REGION/2>& bmp) { return bmp.count(); }
  static uint32_t popcount(const std::bitset<LINES_PER_REGION>& bmp)   { return bmp.count(); }
};

class DSPatchPageBufferEntry {
public:
  uint64_t page_addr, trigger_pc;
  uint32_t trigger_offset;
  std::bitset<LINES_PER_REGION> access_bitmap;
  DSPatchPageBufferEntry(uint64_t p, uint64_t pc, uint32_t off)
    : page_addr(p), trigger_pc(pc), trigger_offset(off) {
    access_bitmap.reset();
    access_bitmap.set(off);
  }
};

class DSPatchSpatialPatternEntry {
public:
  std::bitset<LINES_PER_REGION/2> bmp_covP, bmp_accP;
  DSPatchCounter measure_covP, measure_accP, or_count;
  DSPatchSpatialPatternEntry() : measure_covP(2), measure_accP(2), or_count(2) {
    bmp_covP.reset(); bmp_accP.reset();
  }
};

enum class DSPatchPatternSelection { NONE, COVP, ACCP };

class DSPatchPrefetchEngine {
  std::deque<DSPatchPageBufferEntry> page_buffer_entries;
  DSPatchSpatialPatternEntry spatial_pattern_table[SPT_SIZE];
  uint8_t bandwidth_bucket;

  DSPatchPatternSelection select_pattern_bitmap(const DSPatchSpatialPatternEntry& e, std::bitset<LINES_PER_REGION/2>& sel);
  void train_spatial_pattern_table(const DSPatchPageBufferEntry& evicted);
  void generate_spatial_prefetches(uint64_t pc, uint64_t page_addr, uint32_t trigger_offset,
                           std::vector<uint64_t>& candidates);
public:
  DSPatchPrefetchEngine() : bandwidth_bucket(0) {}
  void update_bw(uint8_t bw)      { bandwidth_bucket = bw; }
  uint8_t get_bandwidth_bucket()   const { return bandwidth_bucket; }
  uint32_t get_pattern_table_index(uint64_t pc);
  void process_access(uint64_t pc, uint64_t addr, std::vector<uint64_t>& candidates);
};

// ============================================================
// Combined SPP + DSPatch Prefetcher
// ============================================================
struct spp_dspatch : public champsim::modules::prefetcher {

  // ── SPP knobs ──────────────────────────────────────────────
  constexpr static bool LOOKAHEAD_ON       = true;
  constexpr static bool FILTER_ON          = true;
  constexpr static bool GHR_ON             = true;
  constexpr static bool SPP_SANITY_CHECK   = true;
  constexpr static bool SPP_DEBUG_PRINT    = false;
  constexpr static bool DSPATCH_BW_DEBUG_PRINT = false;
  constexpr static uint64_t DSPATCH_BW_PRINT_INTERVAL = 100000;

  // ── SPP Signature Table ────────────────────────────────────
  constexpr static std::size_t ST_SET      = 1;
  constexpr static std::size_t ST_WAY      = 256;
  constexpr static unsigned    ST_TAG_BIT  = 16;
  constexpr static unsigned    SIG_SHIFT   = 3;
  constexpr static unsigned    SIG_BIT     = 12;
  constexpr static uint32_t   SIG_MASK    = ((1 << SIG_BIT) - 1);
  constexpr static unsigned    SIG_DELTA_BIT = 7;

  // ── SPP Pattern Table ──────────────────────────────────────
  constexpr static std::size_t PT_SET      = 512;
  constexpr static std::size_t PT_WAY      = 4;
  constexpr static unsigned    C_SIG_BIT   = 4;
  constexpr static unsigned    C_DELTA_BIT = 4;
  constexpr static uint32_t   C_SIG_MAX   = ((1 << C_SIG_BIT)   - 1);
  constexpr static uint32_t   C_DELTA_MAX = ((1 << C_DELTA_BIT) - 1);

  // ── SPP Prefetch Filter ────────────────────────────────────
  constexpr static unsigned    QUOTIENT_BIT  = 10;
  constexpr static unsigned    REMAINDER_BIT = 6;
  constexpr static unsigned    HASH_BIT      = (QUOTIENT_BIT + REMAINDER_BIT + 1);
  constexpr static std::size_t FILTER_SET    = (1 << QUOTIENT_BIT);
  constexpr static uint32_t   FILL_THRESHOLD = 90;
  constexpr static uint32_t   PF_THRESHOLD   = 25;

  // ── SPP Global Register ────────────────────────────────────
  constexpr static unsigned    GLOBAL_COUNTER_BIT = 10;
  constexpr static uint32_t   GLOBAL_COUNTER_MAX  = ((1 << GLOBAL_COUNTER_BIT) - 1);
  constexpr static std::size_t MAX_GHR_ENTRY       = 8;

  using prefetcher::prefetcher;
  uint8_t last_bw_bucket = UINT8_MAX;
  uint64_t bw_sample_count = 0;

  // ChampSim interface
  void     prefetcher_initialize();
  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip,
                                    uint8_t cache_hit, bool useful_prefetch,
                                    access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way,
                                 uint8_t prefetch, champsim::address evicted_addr,
                                 uint32_t metadata_in);
  void prefetcher_cycle_operate();
  void prefetcher_final_stats();

  enum FILTER_REQUEST { SPP_L2C_PREFETCH, SPP_LLC_PREFETCH, L2C_DEMAND, L2C_EVICT };
  static uint64_t get_hash(uint64_t key);

  struct block_in_page_extent : champsim::dynamic_extent {
    block_in_page_extent()
      : dynamic_extent(champsim::data::bits{LOG2_PAGE_SIZE},
                       champsim::data::bits{LOG2_BLOCK_SIZE}) {}
  };
  using offset_type = champsim::address_slice<block_in_page_extent>;

  // ── SPP Signature Table ────────────────────────────────────
  class SIGNATURE_TABLE {
    struct tag_extent : champsim::dynamic_extent {
      tag_extent()
        : dynamic_extent(champsim::data::bits{ST_TAG_BIT + LOG2_PAGE_SIZE},
                         champsim::data::bits{LOG2_PAGE_SIZE}) {}
    };
  public:
    spp_dspatch* _parent;
    using tag_type = champsim::address_slice<tag_extent>;

    bool        valid[ST_SET][ST_WAY];
    tag_type    tag[ST_SET][ST_WAY];
    offset_type last_offset[ST_SET][ST_WAY];
    uint32_t    sig[ST_SET][ST_WAY], lru[ST_SET][ST_WAY];

    SIGNATURE_TABLE() {
      for (uint32_t s = 0; s < ST_SET; s++)
        for (uint32_t w = 0; w < ST_WAY; w++) {
          valid[s][w] = 0; tag[s][w] = tag_type{};
          last_offset[s][w] = offset_type{}; sig[s][w] = 0; lru[s][w] = w;
        }
    }
    void read_and_update_sig(champsim::address addr, uint32_t& last_sig,
                             uint32_t& curr_sig,
                             typename offset_type::difference_type& delta);
  };

  // ── SPP Pattern Table ──────────────────────────────────────
  class PATTERN_TABLE {
  public:
    spp_dspatch* _parent;
    typename offset_type::difference_type delta[PT_SET][PT_WAY];
    uint32_t c_delta[PT_SET][PT_WAY], c_sig[PT_SET];

    PATTERN_TABLE() {
      for (uint32_t s = 0; s < PT_SET; s++) {
        for (uint32_t w = 0; w < PT_WAY; w++) { delta[s][w] = 0; c_delta[s][w] = 0; }
        c_sig[s] = 0;
      }
    }
    void update_pattern(uint32_t last_sig, typename offset_type::difference_type curr_delta);
    void read_pattern(uint32_t curr_sig,
                      std::vector<typename offset_type::difference_type>& delta_q,
                      std::vector<uint32_t>& confidence_q,
                      uint32_t& lookahead_way, uint32_t& lookahead_conf,
                      uint32_t& pf_q_tail, uint32_t& depth);
  };

  // ── SPP Prefetch Filter ────────────────────────────────────
  class PREFETCH_FILTER {
  public:
    spp_dspatch* _parent;
    uint64_t remainder_tag[FILTER_SET];
    bool valid[FILTER_SET], useful[FILTER_SET];

    PREFETCH_FILTER() {
      for (uint32_t s = 0; s < FILTER_SET; s++) {
        remainder_tag[s] = 0; valid[s] = 0; useful[s] = 0;
      }
    }
    bool check(champsim::address pf_addr, FILTER_REQUEST req);
  };

  // ── SPP Global Register ────────────────────────────────────
  class GLOBAL_REGISTER {
  public:
    spp_dspatch* _parent;
    uint32_t    pf_useful, pf_issued, global_accuracy;
    uint8_t     valid[MAX_GHR_ENTRY];
    uint32_t    sig[MAX_GHR_ENTRY], confidence[MAX_GHR_ENTRY];
    offset_type offset[MAX_GHR_ENTRY];
    typename offset_type::difference_type delta[MAX_GHR_ENTRY];

    GLOBAL_REGISTER() : pf_useful(0), pf_issued(0), global_accuracy(0) {
      for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
        valid[i] = 0; sig[i] = 0; confidence[i] = 0;
        offset[i] = offset_type{}; delta[i] = 0;
      }
    }
    void update_entry(uint32_t pf_sig, uint32_t pf_confidence,
                      offset_type pf_offset,
                      typename offset_type::difference_type pf_delta);
    uint32_t check_entry(offset_type page_offset);
  };

  // ── Members ────────────────────────────────────────────────
  DSPatchPrefetchEngine dspatch_engine;
  SIGNATURE_TABLE ST;
  PATTERN_TABLE   PT;
  PREFETCH_FILTER FILTER;
  GLOBAL_REGISTER GHR;
};

#endif // SPP_DSPATCH_H

