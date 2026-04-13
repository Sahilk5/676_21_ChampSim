#include <cstdint>
#include <bitset>
#include <iostream>

#include "dspatch.h"

using namespace std;


uint32_t DSPatchCore::get_spt_index(uint64_t trigger_pc) {
    return (trigger_pc ^ (trigger_pc >> 8) ^ (trigger_pc >> 16)) % SPT_SIZE;
}

void DSPatchCore::update_bw(uint8_t current_bw) {
    bw_bucket = current_bw;
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
    
        // High bandwidth, prioritize accuracy
        if (bw_bucket == 3) {
            if (spt_entry.measure_accP.is_saturated()) {
                selected_bmp.reset();
                return PerfCandidate::NONE;
            } else {
                selected_bmp = spt_entry.bmp_accP;
                return PerfCandidate::ACCP;
            }
        } else if (bw_bucket == 2) { // Moderate bandwidth, use coverage pattern
            if (spt_entry.measure_covP.is_saturated()) {
                selected_bmp = spt_entry.bmp_accP;
                return PerfCandidate::ACCP;
            } else {
                selected_bmp = spt_entry.bmp_covP;
                return PerfCandidate::COVP;
            }
        } else { // Low bandwidth, be aggressive with coverage
            selected_bmp = spt_entry.bmp_covP;
            return PerfCandidate::COVP;
        }
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

    // calculate coverage and accuracy
    uint32_t cov_of_covP = (pop_real == 0) ? 0 : (match_cov * 100 / pop_real);
    uint32_t acc_of_covP = (pop_cov == 0) ? 0 : (match_cov * 100 / pop_cov);
    uint32_t acc_of_accP = (pop_acc == 0) ? 0 : (match_acc * 100 / pop_acc);

    // Modulate CovP
    
    if ((bmp_real_anchored & ~bmp_cov_decomp).any()) {
        spt_entry.or_count.increment();
    }

    if (acc_of_covP < 50 || cov_of_covP < 50) {
        spt_entry.measure_covP.increment();
    }

    if (spt_entry.measure_covP.is_saturated()) {
        spt_entry.bmp_covP = BitMath::compress(bmp_real_anchored);
        spt_entry.measure_covP.reset();
        spt_entry.or_count.reset();
    } else {
        bitset<LINES_PER_REGION> bmp_cov_new = bmp_cov_decomp| bmp_real_anchored;
        spt_entry.bmp_covP = BitMath::compress(bmp_cov_new);
    }

    //Modulate AccP
    if (acc_of_accP < 50) {
        spt_entry.measure_accP.increment();
    } else {
        spt_entry.measure_accP.decrement();
    }

    bitset<LINES_PER_REGION> new_ac_64 = bmp_real_anchored & bmp_cov_decomp;
    spt_entry.bmp_accP = BitMath::compress(new_ac_64);
}

// prefetch time
void DSPatchCore::generate_prefetches(uint64_t pc, uint64_t page_addr,
    uint32_t trigger_offset, std::vector<uint64_t> &prefetch_candidates) {
    
    uint32_t spt_index = get_spt_index(pc);
    SPT_Entry &spt_entry = spt[spt_index];

    bitset<LINES_PER_REGION/2> selected_bmp;
    PerfCandidate selected_pattern = dyn_selecttion(spt_entry, selected_bmp);

    if (selected_pattern == PerfCandidate::NONE) return;

    bitset<LINES_PER_REGION> bmp_decomp = BitMath::decompress(selected_bmp);
    bitset<LINES_PER_REGION> shifted_bmp = BitMath::circular_shift_left(bmp_decomp, trigger_offset);

    // Generate prefetch candidates based on the selected pattern
    for (uint32_t i = 0; i < LINES_PER_REGION; ++i) {
        if (shifted_bmp[i] && i != trigger_offset) {
            uint64_t prefetch_addr = (page_addr * REGION_SIZE_BYTES) + (i * CACHE_LINE_SIZE_BYTES);
            prefetch_candidates.push_back(prefetch_addr);
        }
    }
}


void dspatch::prefetcher_initialize() {
    std::cout<< "Initialized DSPatch Prefetcher" << std::endl;
}

uint32_t dspatch::prefetcher_cache_operate(champsim::address addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in) {
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
    // For now, using L2 MSHR occupancyh as a proxy for bandwidth conditions. This can be replaced with a more direct measure if available.
    double utilization = intern_->get_mshr_occupancy_ratio();

   uint8_t bw_bucket = 0;
    if (utilization > 0.75) {
         bw_bucket = 3; // High bandwidth
    } else if (utilization > 0.5) {
         bw_bucket = 2; // Moderate bandwidth
    } else {
         bw_bucket = 1; // Low bandwidth
    }

    engine.update_bw(bw_bucket);
}

uint32_t dspatch::prefetcher_cache_fill(champsim::address addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in) {
    return metadata_in;
}

void dspatch::prefetcher_final_stats() {
    // TODO
}
