#include <cstdint>
#include <bitset>
#include <iostream>

#include "cache.h"
#include "dspatch.h"
#include "cache.h"
#include "dram_controller.h"

extern uint8_t dram_bw_util_signal;

using namespace std;

namespace {

const char* perf_candidate_name(PerfCandidate candidate) {
    switch (candidate) {
        case PerfCandidate::NONE:
            return "NONE";
        case PerfCandidate::COVP:
            return "COVP";
        case PerfCandidate::ACCP:
            return "ACCP";
    }

    return "UNKNOWN";
}

}  // namespace


uint32_t DSPatchCore::get_spt_index(uint64_t trigger_pc) {
    return (trigger_pc ^ (trigger_pc >> 8) ^ (trigger_pc >> 16)) % SPT_SIZE;
}

/*
    Overall Flow:
    1. Page accessed
    2. anchor it with trigger PC
    3. check if trigger PC has a pattern in SPT
    4. if pattern exists, check if it is good enough to use
    5. if good enough, use the pattern to prefetch
    6. if not good enough, or no pattern, update the pattern with the new
         access and the trigger PC -> Do this when? When the page leaves buffer - for now
*/

PerfCandidate DSPatchCore::dyn_selecttion(const SPT_Entry& spt_entry,
    std::bitset<LINES_PER_REGION/2> &selected_bmp) {
    // High bandwidth, prioritize accuracy.
    if (bw_bucket == 3) {
        if (spt_entry.measure_accP.is_saturated()) {
            selected_bmp.reset();
            return PerfCandidate::NONE;
        }

        selected_bmp = spt_entry.bmp_accP;
        return PerfCandidate::ACCP;
    }

    // Moderate bandwidth, prefer coverage unless CovP has proven bad.
    if (bw_bucket == 2) {
        if (spt_entry.measure_covP.is_saturated()) {
            selected_bmp = spt_entry.bmp_accP;
            return PerfCandidate::ACCP;
        }

        selected_bmp = spt_entry.bmp_covP;
        return PerfCandidate::COVP;
    }

    // Low bandwidth, be aggressive with coverage.
    selected_bmp = spt_entry.bmp_covP;
    return PerfCandidate::COVP;
}

void DSPatchCore::train_spt(const PB_Entry& evicted_entry) {
    uint32_t spt_index = get_spt_index(evicted_entry.trigger_pc);
    SPT_Entry &spt_entry = spt[spt_index];

    // We are taking the access pattern from the evicted page and using it to update the SPT
    // entry corresponding to the trigger PC that caused this page to be brought into the buffer.
    bitset<LINES_PER_REGION> bmp_real_anchored = BitMath::circular_shift_right(
        evicted_entry.access_bitmap, evicted_entry.trigger_offset);

    bitset<LINES_PER_REGION> bmp_cov_decomp = BitMath::decompress(spt_entry.bmp_covP);
    bitset<LINES_PER_REGION> bmp_acc_decomp = BitMath::decompress(spt_entry.bmp_accP);

    uint32_t pop_real = BitMath::popcount(BitMath::compress(bmp_real_anchored));
    uint32_t pop_cov = BitMath::popcount(spt_entry.bmp_covP);
    uint32_t pop_acc = BitMath::popcount(spt_entry.bmp_accP);

    uint32_t match_cov = BitMath::popcount(BitMath::compress(bmp_real_anchored & bmp_cov_decomp));
    uint32_t match_acc = BitMath::popcount(BitMath::compress(bmp_real_anchored & bmp_acc_decomp));

    // Calculate coverage and accuracy.
    uint32_t cov_of_covP = (pop_real == 0) ? 0 : (match_cov * 100 / pop_real);
    uint32_t acc_of_covP = (pop_cov == 0) ? 0 : (match_cov * 100 / pop_cov);
    uint32_t acc_of_accP = (pop_acc == 0) ? 0 : (match_acc * 100 / pop_acc);

    // CovP relearn condition follows the paper more closely:
    // reset only when MeasureCovP saturates and either bandwidth is in the highest quartile
    // or the coverage of CovP is below 50%.
    if (acc_of_covP < 50 || cov_of_covP < 50) {
        spt_entry.measure_covP.increment();
    }

    bool relearn_covP =
        spt_entry.measure_covP.is_saturated() &&
        ((bw_bucket == 3) || (cov_of_covP < 50));

    bool adds_new_bits = (bmp_real_anchored & ~bmp_cov_decomp).any();
    bitset<LINES_PER_REGION> final_cov = bmp_cov_decomp;

    if (relearn_covP) {
        final_cov = bmp_real_anchored;
        spt_entry.measure_covP.reset();
        spt_entry.or_count.reset();
    } else if (adds_new_bits && !spt_entry.or_count.is_saturated()) {
        final_cov = bmp_cov_decomp | bmp_real_anchored;
        spt_entry.or_count.increment();
    }

    spt_entry.bmp_covP = BitMath::compress(final_cov);

    // Modulate AccP.
    if (acc_of_accP < 50) {
        spt_entry.measure_accP.increment();
    } else {
        spt_entry.measure_accP.decrement();
    }

    // AccP is rebuilt from the current anchored program pattern and the final CovP.
    bitset<LINES_PER_REGION> new_acc = bmp_real_anchored & final_cov;
    spt_entry.bmp_accP = BitMath::compress(new_acc);
}

// prefetch time
void DSPatchCore::generate_prefetches(uint64_t pc, uint64_t page_addr,
    uint32_t trigger_offset, std::vector<uint64_t> &prefetch_candidates) {

    uint32_t spt_index = get_spt_index(pc);
    SPT_Entry &spt_entry = spt[spt_index];

    bitset<LINES_PER_REGION/2> selected_bmp;
    PerfCandidate selected_pattern = dyn_selecttion(spt_entry, selected_bmp);

    if constexpr (dspatch::DSPATCH_BW_DEBUG_PRINT) {
        std::cout << "[DSPatch][select] bw=" << unsigned(bw_bucket)
                  << " pc=0x" << std::hex << pc << std::dec
                  << " trigger_offset=" << trigger_offset
                  << " pattern=" << perf_candidate_name(selected_pattern)
                  << '\n';
    }

    if (selected_pattern == PerfCandidate::NONE) return;

    bitset<LINES_PER_REGION> bmp_decomp = BitMath::decompress(selected_bmp);
    bitset<LINES_PER_REGION> shifted_bmp = BitMath::circular_shift_left(bmp_decomp, trigger_offset);

    // Generate prefetch candidates based on the selected pattern.
    for (uint32_t i = 0; i < LINES_PER_REGION; ++i) {
        if (shifted_bmp[i] && i != trigger_offset) {
            uint64_t prefetch_addr = (page_addr * REGION_SIZE_BYTES) + (i * CACHE_LINE_SIZE_BYTES);
            prefetch_candidates.push_back(prefetch_addr);
        }
    }
}

void DSPatchCore::update_bw(uint8_t current_bw) {
    bw_bucket = current_bw;
}


void dspatch::prefetcher_initialize() {
    engine.update_bw(dram_bw_util_signal);
    std::cout<< "Initialized DSPatch Prefetcher" << std::endl;

    if constexpr (DSPATCH_BW_DEBUG_PRINT) {
        last_bw_bucket = dram_bw_util_signal;
        std::cout << "[DSPatch][bw][init] bucket=" << unsigned(last_bw_bucket) << '\n';
    }
}

uint32_t dspatch::prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in) {
    engine.update_bw(dram_bw_util_signal);

    if constexpr (DSPATCH_BW_DEBUG_PRINT) {
        ++bw_sample_count;
        uint8_t current_bw = dram_bw_util_signal;

        if (current_bw != last_bw_bucket) {
            std::cout << "[DSPatch][bw][change] sample=" << bw_sample_count
                      << " bucket=" << unsigned(last_bw_bucket)
                      << "->" << unsigned(current_bw)
                      << " ip=0x" << std::hex << ip.to<uint64_t>()
                      << " addr=0x" << addr.to<uint64_t>() << std::dec
                      << '\n';
            last_bw_bucket = current_bw;
        }
    }

    if (type != access_type::LOAD && type != access_type::RFO) {
        return metadata_in; // Only consider demand accesses for prefetching
    }

    std::vector<uint64_t> prefetch_candidates;
    engine.handle_access(ip.to<uint64_t>(), addr.to<uint64_t>(), prefetch_candidates);

    for(auto pf_addr : prefetch_candidates) {
        bool success = prefetch_line(champsim::address{pf_addr}, true, metadata_in);
        if (!success) {
            break; // Stop issuing prefetches if MSHR is full or prefetch queue is full
        }
    }

    return metadata_in;
}

void dspatch::prefetcher_cycle_operate(){
    engine.update_bw(dram_bw_util_signal);

    if constexpr (DSPATCH_BW_DEBUG_PRINT) {
        ++bw_sample_count;
        if ((bw_sample_count % DSPATCH_BW_PRINT_INTERVAL) == 0) {
            std::cout << "[DSPatch][bw][periodic] sample=" << bw_sample_count
                      << " bucket=" << unsigned(dram_bw_util_signal) << '\n';
        }
    }
}

uint32_t dspatch::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in) {
    return metadata_in;
}

void dspatch::prefetcher_final_stats() {
    // TODO
}

