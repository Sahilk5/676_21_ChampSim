#include <cstdint>
#include <bitset>

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
    uint32_t cov_of_covP = (pop_cov == 0) ? 0 : (match_cov * 100 / pop_cov);
    uint32_t acc_of_covP = (pop_cov == 0) ? 0 : (match_acc * 100 / pop_cov);
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