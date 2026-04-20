#ifndef BOP_H
#define BOP_H

#include <array>
#include <cstdint>

#include "address.h"
#include "modules.h"

struct bop : public champsim::modules::prefetcher {
  // BOP parameters (from HPCA 2016 paper)
  static constexpr int SCORE_MAX   = 31;
  static constexpr int BAD_SCORE   = 10;
  static constexpr int RRT_SIZE    = 256;
  static constexpr int DEFAULT_OFFSET = 1;

  // 5-smooth candidate offsets (prime factors only 2, 3, 5) up to 256
  static constexpr std::array<int, 48> OFFSETS = {
    1,   2,   3,   4,   5,   6,   8,   9,   10,  12,
    15,  16,  18,  20,  24,  25,  27,  30,  32,  36,
    40,  45,  48,  50,  54,  60,  64,  72,  75,  80,
    90,  96,  100, 108, 120, 125, 128, 135, 150, 160,
    180, 192, 200, 216, 225, 240, 250, 256
  };
  static constexpr int NUM_OFFSETS = static_cast<int>(OFFSETS.size());

  // Recent Requests Table
  std::array<champsim::block_number, RRT_SIZE> rrt{};
  std::array<bool, RRT_SIZE> rrt_valid{};
  int rrt_ptr = 0;

  // Scoring state
  std::array<int, NUM_OFFSETS> scores{};
  int  current_idx = 0;
  int  best_offset = DEFAULT_OFFSET;

  // Stats
  uint64_t stat_prefetches_issued  = 0;
  uint64_t stat_prefetches_useful  = 0;
  uint64_t stat_offset_changes     = 0;

  bool rrt_hit(champsim::block_number addr) const;
  void rrt_insert(champsim::block_number addr);
  void reset_learning();

public:
  using champsim::modules::prefetcher::prefetcher;

  uint32_t prefetcher_cache_operate(champsim::address addr, champsim::address ip,
                                    uint8_t cache_hit, bool useful_prefetch,
                                    access_type type, uint32_t metadata_in);
  uint32_t prefetcher_cache_fill(champsim::address addr, long set, long way,
                                 uint8_t prefetch, champsim::address evicted_addr,
                                 uint32_t metadata_in);
  void prefetcher_final_stats();
};

#endif
