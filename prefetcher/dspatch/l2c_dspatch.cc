#include <iostream>

#include "dspatch.h"
#include "cache.h"

namespace {
    DSPatchCore dspatch_engine;
}

void CACHE::prefetcher_initalize() {
    std::cout<< "Initialized DSPatch Prefetcher" << std::endl;
}

uint32_t CACHE::prefetcher_cache_operate(uint64_t addr, champsim::address ip, bool cache_hit, bool useful_prefetch, access_type type, uint32_t metadata_in) {
    if (type != LOAD && type != RFO) {
        return metadata_in; // Only consider demand accesses for prefetching
    }

    std::vector<uint64_t> prefetch_candidates;
    dspatch_engine.handle_access(ip, addr, prefetch_candidates);

    for(auto pf_addr : prefetch_candidates) {
        bool success = prefetch_line(pf_addr, true, metadata_in);
        if (!success) {
            break; // Stop issuing prefetches if MSHR is full or prefetch queue is full
        }
    }

    return metadata_in;
}

void CACHE::prefetcher_cycle_operate(){
    // For now, using L2 MSHR occupancyh as a proxy for bandwidth conditions. This can be replaced with a more direct measure if available.
    uint32_t mshr_occupancy = get_mshr_occupancy();
    uint32_t mshr_size = get_mshr_size();

   float utilization = (mshr_size == 0) ? 0 : (static_cast<float>(mshr_occupancy) / mshr_size);

   uint8_t bw_bucket = 0;
    if (utilization > 0.75) {
         bw_bucket = 3; // High bandwidth
    } else if (utilization > 0.5) {
         bw_bucket = 2; // Moderate bandwidth
    } else {
         bw_bucket = 1; // Low bandwidth
    }

    dspatch_engine.update_bw(bw_bucket);
}

uint32_t CACHE::prefetcher_cache_fill(uint64_t addr, long set, long way, uint8_t prefetch, champsim::address evicted_addr, uint32_t metadata_in) {
    return metadata_in;
}

void CACHE::prefetcher_final_stats() {
    // TODO
}