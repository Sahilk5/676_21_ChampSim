#include "spp_dspatch.h"
#include "dram_controller.h"

#include <cassert>
#include <iostream>
#include <unordered_set>

extern uint8_t dram_bw_util_signal;

uint8_t bw_bucket = dram_bw_util_signal; 

// ============================================================
// DSPatch Core Implementation
// ============================================================

uint32_t DSPatchPrefetchEngine::get_pattern_table_index(uint64_t pc) {
  return (pc ^ (pc >> 8) ^ (pc >> 16)) % SPT_SIZE;
}

DSPatchPatternSelection DSPatchPrefetchEngine::select_pattern_bitmap(const DSPatchSpatialPatternEntry& e,
                                         std::bitset<LINES_PER_REGION/2>& sel) {
  if (bandwidth_bucket == 3) {            // High pressure → accuracy-only
    if (e.measure_accP.is_saturated()) { sel.reset(); return DSPatchPatternSelection::NONE; }
    sel = e.bmp_accP; return DSPatchPatternSelection::ACCP;
  } else if (bandwidth_bucket == 2) {     // Moderate → prefer coverage, fall back to acc
    if (e.measure_covP.is_saturated()) { sel = e.bmp_accP; return DSPatchPatternSelection::ACCP; }
    sel = e.bmp_covP; return DSPatchPatternSelection::COVP;
  } else {                         // Low pressure → aggressive coverage
    sel = e.bmp_covP; return DSPatchPatternSelection::COVP;
  }
}

void DSPatchPrefetchEngine::train_spatial_pattern_table(const DSPatchPageBufferEntry& evicted) {
  uint32_t idx = get_pattern_table_index(evicted.trigger_pc);
  DSPatchSpatialPatternEntry& e = spatial_pattern_table[idx];

  auto bmp_anchored = DSPatchBitmapOps::circular_shift_right(evicted.access_bitmap,
                                                    evicted.trigger_offset);
  auto bmp_cov_dec  = DSPatchBitmapOps::decompress(e.bmp_covP);
  auto bmp_acc_dec  = DSPatchBitmapOps::decompress(e.bmp_accP);

  uint32_t pop_real = DSPatchBitmapOps::popcount(DSPatchBitmapOps::compress(bmp_anchored));
  uint32_t pop_cov  = DSPatchBitmapOps::popcount(e.bmp_covP);
  uint32_t pop_acc  = DSPatchBitmapOps::popcount(e.bmp_accP);

  uint32_t match_cov = DSPatchBitmapOps::popcount(DSPatchBitmapOps::compress(bmp_anchored & bmp_cov_dec));
  uint32_t match_acc = DSPatchBitmapOps::popcount(DSPatchBitmapOps::compress(bmp_anchored & bmp_acc_dec));

  uint32_t cov_of_covP = pop_real ? (match_cov * 100 / pop_real) : 0;
  uint32_t acc_of_covP = pop_cov  ? (match_cov * 100 / pop_cov)  : 0;
  uint32_t acc_of_accP = pop_acc  ? (match_acc * 100 / pop_acc)  : 0;

  // Update CovP
  if ((bmp_anchored & ~bmp_cov_dec).any()) e.or_count.increment();
  if (acc_of_covP < 50 || cov_of_covP < 50) {
    e.measure_covP.increment();
    if (e.measure_covP.is_saturated()) {
      e.bmp_covP = DSPatchBitmapOps::compress(bmp_anchored);
      e.measure_covP.reset(); e.or_count.reset();
    } else {
      e.bmp_covP = DSPatchBitmapOps::compress(bmp_cov_dec | bmp_anchored);
    }
  }

  // Update AccP
  if (acc_of_accP < 50) e.measure_accP.increment();
  else                   e.measure_accP.decrement();
  e.bmp_accP = DSPatchBitmapOps::compress(bmp_anchored & bmp_cov_dec);
}

void DSPatchPrefetchEngine::generate_spatial_prefetches(uint64_t pc, uint64_t page_addr,
                                      uint32_t trigger_offset,
                                      std::vector<uint64_t>& candidates) {
  uint32_t idx = get_pattern_table_index(pc);
  DSPatchSpatialPatternEntry& e = spatial_pattern_table[idx];

  std::bitset<LINES_PER_REGION/2> selected_bmp;
  DSPatchPatternSelection pattern = select_pattern_bitmap(e, selected_bmp);
  if (pattern == DSPatchPatternSelection::NONE) return;

  auto bmp_full    = DSPatchBitmapOps::decompress(selected_bmp);
  auto shifted_bmp = DSPatchBitmapOps::circular_shift_left(bmp_full, trigger_offset);

  for (uint32_t i = 0; i < LINES_PER_REGION; ++i) {
    if (shifted_bmp[i] && i != trigger_offset)
      candidates.push_back(page_addr * REGION_SIZE_BYTES + i * CACHE_LINE_SIZE_BYTES);
  }
}

void DSPatchPrefetchEngine::process_access(uint64_t pc, uint64_t addr,
                                std::vector<uint64_t>& candidates) {
  uint64_t page_addr = addr / REGION_SIZE_BYTES;
  uint32_t offset    = (addr % REGION_SIZE_BYTES) / CACHE_LINE_SIZE_BYTES;

  auto it = std::find_if(page_buffer_entries.begin(), page_buffer_entries.end(),
    [page_addr](const DSPatchPageBufferEntry& e){ return e.page_addr == page_addr; });

  if (it != page_buffer_entries.end()) {
    it->access_bitmap.set(offset);
    DSPatchPageBufferEntry tmp = *it; page_buffer_entries.erase(it); page_buffer_entries.push_back(tmp);
  } else {
    if (page_buffer_entries.size() >= PB_SIZE) {
      train_spatial_pattern_table(page_buffer_entries.front());
      page_buffer_entries.pop_front();
    }
    page_buffer_entries.emplace_back(page_addr, pc, offset);
    generate_spatial_prefetches(pc, page_addr, offset, candidates);
  }
}

// ============================================================
// SPP + DSPatch Combined Module
// ============================================================

uint64_t spp_dspatch::get_hash(uint64_t key) {
  key += (key << 12); key ^= (key >> 22); key += (key << 4);
  key ^= (key >>  9); key += (key << 10); key ^= (key >>  2);
  key += (key <<  7); key ^= (key >> 12);
  key = (key >> 3) * 2654435761ULL;
  return key;
}

void spp_dspatch::prefetcher_initialize() {
  std::cout << "[SPP+DSPatch] Initialized combined prefetcher\n";
  std::cout << "  SPP ST_WAY=" << ST_WAY << " PT_SET=" << PT_SET
            << " PF_THRESHOLD=" << PF_THRESHOLD
            << " FILL_THRESHOLD=" << FILL_THRESHOLD << "\n";
  std::cout << "  DSPatch PB_SIZE=" << PB_SIZE << " SPT_SIZE=" << SPT_SIZE << "\n";
  ST._parent = this; PT._parent = this;
  FILTER._parent = this; GHR._parent = this;
}

// ── cycle_operate: update bandwidth bucket (shared by both engines) ─────────
void spp_dspatch::prefetcher_cycle_operate() {
  dspatch_engine.update_bw(bw_bucket);
}

// ── Combined cache_operate ───────────────────────────────────────────────────
uint32_t spp_dspatch::prefetcher_cache_operate(champsim::address addr,
                                               champsim::address ip,
                                               uint8_t cache_hit,
                                               bool useful_prefetch,
                                               access_type type,
                                               uint32_t metadata_in) {
  if (type != access_type::LOAD && type != access_type::RFO)
    return metadata_in;

  champsim::page_number page{addr};

  // ── Initialise SPP per-access state ──────────────────────────────────────
  uint32_t last_sig = 0, curr_sig = 0, depth = 0;
  typename offset_type::difference_type delta = 0;

  const auto queue_capacity = std::max<std::size_t>(1, intern_->MSHR_SIZE);
  std::vector<uint32_t> confidence_q(queue_capacity, 0);
  std::vector<typename offset_type::difference_type> delta_q(queue_capacity, 0);
  confidence_q[0] = 100;

  GHR.global_accuracy = GHR.pf_issued
    ? ((100 * GHR.pf_useful) / GHR.pf_issued) : 0;

  // ── SPP Stage 1: read/update Signature Table ─────────────────────────────
  ST.read_and_update_sig(addr, last_sig, curr_sig, delta);

  // ── SPP + Filter: mark demand access, update accuracy counters ───────────
  FILTER.check(addr, L2C_DEMAND);

  // ── SPP Stage 2: update Pattern Table ────────────────────────────────────
  if (last_sig)
    PT.update_pattern(last_sig, delta);

  // ── DSPatch: gather intra-page spatial candidates ─────────────────────────
  std::vector<uint64_t> ds_candidates;
  dspatch_engine.process_access(ip.to<uint64_t>(), addr.to<uint64_t>(), ds_candidates);

  // ── SPP Stage 3: lookahead — collect (addr, fill_l2) pairs ───────────────
  // When bandwidth pressure is high (bandwidth_bucket==3), raise SPP's effective
  // threshold so it issues only very high-confidence prefetches, complementing
  // DSPatch's AccP fallback at the same bandwidth condition.
  uint32_t effective_pf_thresh =
    (dspatch_engine.get_bandwidth_bucket() >= 3) ? FILL_THRESHOLD : PF_THRESHOLD;

  struct SPP_PF { champsim::address addr; bool fill_l2; };
  std::vector<SPP_PF> spp_pf_list;

  auto base_addr = addr;
  uint32_t lookahead_conf = 100, pf_q_head = 0, pf_q_tail = 0;
  bool do_lookahead = false;

  do {
    uint32_t lookahead_way = PT_WAY;
    PT.read_pattern(curr_sig, delta_q, confidence_q,
                    lookahead_way, lookahead_conf, pf_q_tail, depth);

    do_lookahead = false;
    for (uint32_t i = pf_q_head; i < pf_q_tail; i++) {
      if (confidence_q[i] >= effective_pf_thresh) {
        champsim::address pf_addr{champsim::block_number{base_addr} + delta_q[i]};
        bool fill_l2 = (confidence_q[i] >= FILL_THRESHOLD);

        if (champsim::page_number{pf_addr} == page) {
          // Intra-page: check filter before queueing
          FILTER_REQUEST freq = fill_l2 ? SPP_L2C_PREFETCH : SPP_LLC_PREFETCH;
          if (FILTER.check(pf_addr, freq)) {
            spp_pf_list.push_back({pf_addr, fill_l2});
            if (fill_l2) {
              GHR.pf_issued++;
              if (GHR.pf_issued > GLOBAL_COUNTER_MAX) {
                GHR.pf_issued >>= 1; GHR.pf_useful >>= 1;
              }
            }
          }
        } else {
          // Cross-page: store in GHR to bootstrap next page's ST miss
          if constexpr (GHR_ON)
            GHR.update_entry(curr_sig, confidence_q[i],
                             offset_type{pf_addr}, delta_q[i]);
        }

        do_lookahead = true;
        pf_q_head++;
      }
    }

    // Advance base address and speculative signature for next lookahead depth
    if (lookahead_way < PT_WAY) {
      uint32_t set = get_hash(curr_sig) % PT_SET;
      base_addr += (PT.delta[set][lookahead_way] << LOG2_BLOCK_SIZE);
      auto sig_delta = (PT.delta[set][lookahead_way] < 0)
        ? ((-1 * PT.delta[set][lookahead_way]) + (1 << (SIG_DELTA_BIT - 1)))
        : PT.delta[set][lookahead_way];
      curr_sig = ((curr_sig << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
    }
  } while (LOOKAHEAD_ON && do_lookahead);

  // ── Merge & issue: deduplicate across both engines ────────────────────────
  // Issue SPP candidates first (carry explicit confidence + filter state).
  // Then fill remaining coverage with DSPatch spatial candidates.
  std::unordered_set<uint64_t> issued;

  for (auto& pf : spp_pf_list) {
    if (issued.insert(pf.addr.to<uint64_t>()).second) {
      if (!prefetch_line(pf.addr, pf.fill_l2, 0))
        break;  // MSHR / prefetch queue full
    }
  }

  for (uint64_t pf_raw : ds_candidates) {
    if (issued.insert(pf_raw).second) {
      if (!prefetch_line(champsim::address{pf_raw}, true, metadata_in))
        break;
    }
  }

  return metadata_in;
}

// ── cache_fill: maintain filter accuracy counters ───────────────────────────
uint32_t spp_dspatch::prefetcher_cache_fill(champsim::address addr, long set,
                                            long way, uint8_t prefetch,
                                            champsim::address evicted_addr,
                                            uint32_t metadata_in) {
  if constexpr (FILTER_ON)
    FILTER.check(evicted_addr, L2C_EVICT);
  return metadata_in;
}

void spp_dspatch::prefetcher_final_stats() {
  std::cout << "[SPP+DSPatch] pf_issued=" << GHR.pf_issued
            << " pf_useful=" << GHR.pf_useful
            << " global_accuracy=" << GHR.global_accuracy << "\n";
}

// ============================================================
// SPP Signature Table
// ============================================================
void spp_dspatch::SIGNATURE_TABLE::read_and_update_sig(
    champsim::address addr, uint32_t& last_sig, uint32_t& curr_sig,
    typename offset_type::difference_type& delta) {

  auto set          = get_hash(champsim::page_number{addr}.to<uint64_t>()) % ST_SET;
  auto match        = ST_WAY;
  tag_type  partial_page{addr};
  offset_type page_offset{addr};
  bool ST_hit       = false;
  long sig_delta    = 0;

  // Case 1: ST hit
  for (match = 0; match < ST_WAY; match++) {
    if (valid[set][match] && (tag[set][match] == partial_page)) {
      last_sig = sig[set][match];
      delta    = champsim::offset(last_offset[set][match], page_offset);
      if (delta) {
        sig_delta = (delta < 0)
          ? ((-1 * delta) + (1 << (SIG_DELTA_BIT - 1))) : delta;
        sig[set][match] = ((last_sig << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
        curr_sig = sig[set][match];
        last_offset[set][match] = page_offset;
      } else { last_sig = 0; }
      ST_hit = true; break;
    }
  }

  // Case 2: ST miss — find invalid way
  if (match == ST_WAY) {
    for (match = 0; match < ST_WAY; match++) {
      if (!valid[set][match]) {
        valid[set][match] = 1; tag[set][match] = partial_page;
        sig[set][match] = 0;  curr_sig = 0;
        last_offset[set][match] = page_offset; break;
      }
    }
  }

  // Case 3: Evict LRU way
  if constexpr (SPP_SANITY_CHECK) {
    if (match == ST_WAY) {
      for (match = 0; match < ST_WAY; match++) {
        if (lru[set][match] == ST_WAY - 1) {
          tag[set][match] = partial_page;
          sig[set][match] = 0; curr_sig = 0;
          last_offset[set][match] = page_offset; break;
        }
      }
      if (match == ST_WAY) { std::cout << "[ST] No victim!\n"; assert(0); }
    }
  }

  // GHR bootstrap on ST miss
  if constexpr (GHR_ON) {
    if (!ST_hit) {
      uint32_t ghr_way = _parent->GHR.check_entry(page_offset);
      if (ghr_way < MAX_GHR_ENTRY) {
        long gd = _parent->GHR.delta[ghr_way];
        sig_delta = (gd < 0) ? ((-1*gd) + (1 << (SIG_DELTA_BIT-1))) : gd;
        sig[set][match] = ((_parent->GHR.sig[ghr_way] << SIG_SHIFT) ^ sig_delta) & SIG_MASK;
        curr_sig = sig[set][match];
      }
    }
  }

  // LRU update
  for (uint32_t w = 0; w < ST_WAY; w++) {
    if (lru[set][w] < lru[set][match]) lru[set][w]++;
    if constexpr (SPP_SANITY_CHECK) {
      if (lru[set][w] >= ST_WAY) {
        std::cout << "[ST] LRU overflow set=" << set << " way=" << w << "\n"; assert(0);
      }
    }
  }
  lru[set][match] = 0;
}

// ============================================================
// SPP Pattern Table
// ============================================================
void spp_dspatch::PATTERN_TABLE::update_pattern(
    uint32_t last_sig, typename offset_type::difference_type curr_delta) {

  uint32_t set = get_hash(last_sig) % PT_SET, match = 0;

  // Hit
  for (match = 0; match < PT_WAY; match++) {
    if (delta[set][match] == curr_delta) {
      c_delta[set][match]++; c_sig[set]++;
      if (c_sig[set] > C_SIG_MAX) {
        for (uint32_t w = 0; w < PT_WAY; w++) c_delta[set][w] >>= 1;
        c_sig[set] >>= 1;
      }
      return;
    }
  }

  // Miss — replace minimum c_delta entry
  uint32_t victim = PT_WAY, min_c = C_SIG_MAX;
  for (match = 0; match < PT_WAY; match++) {
    if (c_delta[set][match] < min_c) { victim = match; min_c = c_delta[set][match]; }
  }
  if constexpr (SPP_SANITY_CHECK) {
    if (victim == PT_WAY) { std::cout << "[PT] No victim!\n"; assert(0); }
  }
  delta[set][victim] = curr_delta; c_delta[set][victim] = 0; c_sig[set]++;
  if (c_sig[set] > C_SIG_MAX) {
    for (uint32_t w = 0; w < PT_WAY; w++) c_delta[set][w] >>= 1;
    c_sig[set] >>= 1;
  }
}

void spp_dspatch::PATTERN_TABLE::read_pattern(
    uint32_t curr_sig,
    std::vector<typename offset_type::difference_type>& delta_q,
    std::vector<uint32_t>& confidence_q,
    uint32_t& lookahead_way, uint32_t& lookahead_conf,
    uint32_t& pf_q_tail, uint32_t& depth) {

  const uint32_t max_entries = static_cast<uint32_t>(std::min(delta_q.size(), confidence_q.size()));
  if (max_entries == 0 || pf_q_tail >= max_entries) {
    lookahead_conf = 0;
    return;
  }

  uint32_t set = get_hash(curr_sig) % PT_SET;
  uint32_t local_conf = 0, pf_conf = 0, max_conf = 0;

  if (c_sig[set]) {
    for (uint32_t w = 0; w < PT_WAY && pf_q_tail < max_entries; w++) {
      local_conf = (100 * c_delta[set][w]) / c_sig[set];
      pf_conf    = depth
        ? (_parent->GHR.global_accuracy * c_delta[set][w] / c_sig[set]
           * lookahead_conf / 100)
        : local_conf;

      if (pf_conf >= PF_THRESHOLD) {
        confidence_q[pf_q_tail] = pf_conf;
        delta_q[pf_q_tail]      = delta[set][w];
        if (pf_conf > max_conf) { lookahead_way = w; max_conf = pf_conf; }
      } else {
        confidence_q[pf_q_tail] = 0;
        delta_q[pf_q_tail]      = 0;
      }
      pf_q_tail++;
    }
  } else {
    confidence_q[pf_q_tail] = 0;
    delta_q[pf_q_tail]      = 0;
    pf_q_tail++;
  }

  lookahead_conf = max_conf;
  if (lookahead_conf >= PF_THRESHOLD) depth++;
}

// ============================================================
// SPP Prefetch Filter
// ============================================================
bool spp_dspatch::PREFETCH_FILTER::check(champsim::address check_addr,
                                         FILTER_REQUEST req) {
  champsim::block_number cache_line{check_addr};
  auto hash      = get_hash(cache_line.to<uint64_t>());
  auto quotient  = (hash >> REMAINDER_BIT) & ((1 << QUOTIENT_BIT) - 1);
  auto remainder =  hash % (1 << REMAINDER_BIT);

  switch (req) {
    case spp_dspatch::SPP_L2C_PREFETCH:
      if ((valid[quotient] || useful[quotient]) && remainder_tag[quotient] == remainder)
        return false;
      valid[quotient] = 1; useful[quotient] = 0; remainder_tag[quotient] = remainder;
      break;

    case spp_dspatch::SPP_LLC_PREFETCH:
      if ((valid[quotient] || useful[quotient]) && remainder_tag[quotient] == remainder)
        return false;
      // Do NOT set valid for LLC-fill prefetches (intentional — see spp_dev comment)
      break;

    case spp_dspatch::L2C_DEMAND:
      if (remainder_tag[quotient] == remainder && !useful[quotient]) {
        useful[quotient] = 1;
        if (valid[quotient]) _parent->GHR.pf_useful++;
      }
      break;

    case spp_dspatch::L2C_EVICT:
      if (valid[quotient] && !useful[quotient] && _parent->GHR.pf_useful)
        _parent->GHR.pf_useful--;
      valid[quotient] = 0; useful[quotient] = 0; remainder_tag[quotient] = 0;
      break;

    default:
      std::cout << "[FILTER] Invalid request " << req << "\n"; assert(0);
  }
  return true;
}

// ============================================================
// SPP Global History Register
// ============================================================
void spp_dspatch::GLOBAL_REGISTER::update_entry(
    uint32_t pf_sig, uint32_t pf_confidence,
    offset_type pf_offset,
    typename offset_type::difference_type pf_delta) {

  uint32_t min_conf = 100, victim = MAX_GHR_ENTRY;
  for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
    if (valid[i] && offset[i] == pf_offset) {
      sig[i] = pf_sig; confidence[i] = pf_confidence; delta[i] = pf_delta;
      return;
    }
    if (confidence[i] < min_conf) { min_conf = confidence[i]; victim = i; }
  }
  if (victim >= MAX_GHR_ENTRY) { std::cout << "[GHR] No victim!\n"; assert(0); }
  valid[victim] = 1; sig[victim] = pf_sig; confidence[victim] = pf_confidence;
  offset[victim] = pf_offset; delta[victim] = pf_delta;
}

uint32_t spp_dspatch::GLOBAL_REGISTER::check_entry(offset_type page_offset) {
  uint32_t max_conf = 0, best = MAX_GHR_ENTRY;
  for (uint32_t i = 0; i < MAX_GHR_ENTRY; i++) {
    if (offset[i] == page_offset && max_conf < confidence[i]) {
      max_conf = confidence[i]; best = i;
    }
  }
  return best;
}

