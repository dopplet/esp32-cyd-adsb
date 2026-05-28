#pragma once
#include "aircraft.h"

// Connection type
enum NetType { NET_NONE, NET_ETHERNET, NET_WIFI };

// Initialize network and start the background fetch task
void fetcher_init(AircraftList *list);

// Returns true if any network is connected
bool fetcher_wifi_connected();

// Returns which network is currently active
NetType fetcher_connection_type();

// Returns the timestamp of the last successful fetch
uint32_t fetcher_last_update();

// Network stats
struct FetcherStats {
    uint32_t fetch_ok;
    uint32_t fetch_fail;
    uint32_t enrich_ok;
    uint32_t enrich_fail;
    uint32_t bytes_received;
    uint32_t last_fetch_ms;     // duration of last successful fetch
    char ip_addr[16];
};
const FetcherStats* fetcher_get_stats();

// Request a one-shot fetch of the closest aircraft to the given point.
// Callback is invoked from the fetcher task with the ICAO hex of the result.
typedef void (*ClosestCb)(const char *icao_hex);
void fetcher_request_closest(float lat, float lon, int radius_nm, ClosestCb cb);
