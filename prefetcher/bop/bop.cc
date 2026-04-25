#include "bop.h"

#include <cstdio>
#include <iostream>

#include "cache.h"
#include "dram_controller.h"

extern uint8_t dram_bw_util_signal;

bool bop::rrt_hit(champsim::block_number addr) const
{
  for (int i = 0; i < RRT_SIZE; i++)
    if (rrt_valid[i] && rrt[i] == addr)
      return true;
  return false;
}

void bop::rrt_insert(champsim::block_number addr)
{
  rrt[rrt_ptr]       = addr;
  rrt_valid[rrt_ptr] = true;
  rrt_ptr            = (rrt_ptr + 1) % RRT_SIZE;
}

void bop::reset_learning()
{
  scores.fill(0);
  current_idx = 0;
}

void bop::prefetcher_initialize()
{
  if constexpr (BOP_BW_DEBUG_PRINT) {
    last_bw_bucket = dram_bw_util_signal;
    std::cout << "[BOP][bw][init] bucket=" << unsigned(last_bw_bucket) << '\n';
  }
}

void bop::prefetcher_cycle_operate()
{
  if constexpr (BOP_BW_DEBUG_PRINT) {
    ++bw_sample_count;
    if ((bw_sample_count % BOP_BW_PRINT_INTERVAL) == 0) {
      std::cout << "[BOP][bw][periodic] sample=" << bw_sample_count
                << " bucket=" << unsigned(dram_bw_util_signal)
                << " best_offset=" << best_offset
                << '\n';
    }
  }
}

uint32_t bop::prefetcher_cache_operate(champsim::address addr, champsim::address ip,
                                       uint8_t cache_hit, bool useful_prefetch,
                                       access_type type, uint32_t metadata_in)
{
  if constexpr (BOP_BW_DEBUG_PRINT) {
    ++bw_sample_count;
    uint8_t current_bw = dram_bw_util_signal;

    if (current_bw != last_bw_bucket) {
      std::cout << "[BOP][bw][change] sample=" << bw_sample_count
                << " bucket=" << unsigned(last_bw_bucket)
                << "->" << unsigned(current_bw)
                << " ip=0x" << std::hex << ip.to<uint64_t>()
                << " addr=0x" << addr.to<uint64_t>() << std::dec
                << " best_offset=" << best_offset
                << '\n';
      last_bw_bucket = current_bw;
    }
  }

  champsim::block_number bl{addr};

  // Score the current candidate offset: was (bl - offset) accessed recently?
  auto test_addr = bl - static_cast<champsim::block_number::difference_type>(OFFSETS[current_idx]);
  if (rrt_hit(test_addr))
    scores[current_idx]++;

  // Record this access in the RRT
  rrt_insert(bl);

  // Count useful prefetches (demand hit on a previously prefetched line)
  if (useful_prefetch)
    stat_prefetches_useful++;

  // Issue prefetch with the current best offset
  champsim::address pf_addr{bl + static_cast<champsim::block_number::difference_type>(best_offset)};
  if (intern_->virtual_prefetch || champsim::page_number{pf_addr} == champsim::page_number{addr}) {
    bool issued = prefetch_line(pf_addr, true, metadata_in);
    if (issued)
      stat_prefetches_issued++;
  }

  // Immediately promote this offset if its score hit SCORE_MAX
  if (scores[current_idx] >= SCORE_MAX) {
    if (OFFSETS[current_idx] != best_offset)
      stat_offset_changes++;
    best_offset = OFFSETS[current_idx];
    reset_learning();
    return metadata_in;
  }

  // Advance to the next candidate offset (round-robin)
  current_idx = (current_idx + 1) % NUM_OFFSETS;

  // End of a full round: pick the best-scoring offset
  if (current_idx == 0) {
    int max_score = 0;
    int max_idx   = 0;
    for (int i = 0; i < NUM_OFFSETS; i++) {
      if (scores[i] > max_score) {
        max_score = scores[i];
        max_idx   = i;
      }
    }
    int new_offset = (max_score >= BAD_SCORE) ? OFFSETS[max_idx] : DEFAULT_OFFSET;
    if (new_offset != best_offset)
      stat_offset_changes++;
    best_offset = new_offset;
    reset_learning();
  }

  return metadata_in;
}

uint32_t bop::prefetcher_cache_fill(champsim::address addr, long set, long way,
                                    uint8_t prefetch, champsim::address evicted_addr,
                                    uint32_t metadata_in)
{
  // Insert prefetched lines into the RRT so they can score future offsets
  if (prefetch)
    rrt_insert(champsim::block_number{addr});
  return metadata_in;
}

void bop::prefetcher_final_stats()
{
  double accuracy = (stat_prefetches_issued > 0)
                    ? 100.0 * stat_prefetches_useful / stat_prefetches_issued
                    : 0.0;

  printf("\n=== BOP Prefetcher Stats ===\n");
  printf("  Prefetches issued  : %lu\n", stat_prefetches_issued);
  printf("  Prefetches useful  : %lu\n", stat_prefetches_useful);
  printf("  Prefetch accuracy  : %.2f%%\n", accuracy);
  printf("  Best offset changes: %lu\n", stat_offset_changes);
  printf("  Final best offset  : %d cache lines\n", best_offset);
  printf("============================\n\n");
}

