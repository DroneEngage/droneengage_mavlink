# UDP Proxy Starlink/Satellite Network Hardening

## Problem Statement

The UDP proxy communication between the Node.js server (`droneengage_server`) and C++ drone module (`droneengage_mavlink`) works perfectly on normal Internet connections but experiences issues on satellite networks (Starlink/starnet) due to:

1. **NAT Mapping Expiry**: No periodic traffic from the C++ drone side causes NAT mappings to expire (30-120s typical), making Node's "last-caller" tuple go stale
2. **Blocking I/O**: C++ client uses blocking `recvfrom()` with `MSG_WAITALL`, causing indefinite hangs on high-latency satellite links
3. **DNS Resolution Issues**: Synchronous DNS resolution can block during satellite DNS outages
4. **Buffer Mismatch**: Server uses 4MB buffers while client uses OS defaults, causing packet loss under burst conditions
5. **No Connection Resilience**: No heartbeat or reconnection mechanism for frequent satellite drops

## Solution Implemented

### C++ UDP Proxy Client Changes (`droneengage_mavlink/src/udp_proxy/`)

#### Header (`udpProxy.hpp`)
- Added chrono include for timeout handling
- Added timeout constants:
  - `UDP_RCV_TIMEOUT_MS = 1500ms` - Receive timeout
  - `UDP_SND_TIMEOUT_MS = 1500ms` - Send timeout  
  - `UDP_KEEPALIVE_MS = 15000ms` - Keepalive interval
  - `UDP_BUF_SIZE = 4MB` - Socket buffer size
- Added member variables for keepalive timing and DNS resolution tracking
- Added method declarations for `setupSocketOptions()` and `sendKeepAlive()`

#### Implementation (`udpProxy.cpp`)
- **Fixed pthread priority**: Used proper `pthread_setschedparam` instead of deprecated `pthread_setschedprio`
- **Socket options setup**: Added `SO_REUSEADDR`, `SO_RCVTIMEO`, `SO_SNDTIMEO`, `SO_RCVBUF`, `SO_SNDBUF` for 4MB buffers and 1.5s timeouts
- **Improved DNS resolution**: IP-first approach using `inet_pton`, fallback to `getaddrinfo()` for hostnames
- **Non-blocking receiver**: 
  - Removed `MSG_WAITALL` (has no effect for UDP)
  - Added timeout handling with `EAGAIN/EWOULDBLOCK` detection
  - Sends keepalive when timeout occurs
- **Buffer overflow fix**: Safe null-terminator clamping `buffer[std::min(n, MAXLINE - 1)] = 0` to prevent off-by-one errors
- **Keepalive mechanism**: Sends 1-byte `0x00` packet every 15s when idle
- **Safer thread joining**: Added `joinable()` check before joining threads

### Node.js UDP Proxy Server Changes (`droneengage_server/server/js_udp_proxy.js`)

#### Message Handling Improvements
- **Keepalive swallowing**: Detects and drops 1-byte `0x00` packets (doesn't forward to peers)
- **Peer change logging**: Logs when UDP peer changes (helps diagnose Starlink port rebinding)
- **Error-aware sending**: Added error callback to detect send buffer saturation

#### Connection Management
- **Proactive keepalives**: Sends 1-byte keepalives to both peers every 15s
- **Timer cleanup**: Properly cleans up keepalive timer on close

## Key Benefits for Starlink/Satellite Networks

1. **NAT Mapping Preservation**: 15s keepalives prevent NAT expiry (typical 30-120s timers)
2. **No Blocking Operations**: 1.5s timeouts prevent indefinite hangs on high-latency links
3. **Buffer Matching**: 4MB buffers on both sides handle bursty loss/RTT conditions
4. **DNS Resilience**: IP-first resolution avoids DNS delays on satellite links
5. **Clean MAVLink Stream**: Keepalives are swallowed by proxy, never reach GCS/FCB
6. **Error Visibility**: Peer change logging helps diagnose Starlink port rebinding
7. **Connection Resilience**: System recovers automatically from satellite drops

## Technical Details

### Keepalive Protocol
- **Format**: Single byte `0x00`
- **Frequency**: Every 15 seconds when idle
- **Direction**: Bidirectional (drone → server, server → both peers)
- **Handling**: Server swallows keepalives, never forwards to application layer

### Timeout Configuration
- **Receive/Send timeouts**: 1500ms (accommodates Starlink RTT up to ~1500ms)
- **Keepalive interval**: 15000ms (well below typical NAT expiry of 30-120s)
- **Socket buffers**: 4MB (matches between client and server)

### DNS Resolution Strategy
1. Try `inet_pton()` first (IP addresses)
2. Fall back to `getaddrinfo(AF_INET)` for hostnames
3. Fail fast with clear error messages if DNS unavailable

## Testing Recommendations

### Network Simulation
Use `tc` (traffic control) to simulate satellite conditions:
```bash
# High latency and jitter
tc qdisc add dev eth0 root netem delay 1200ms 300ms

# Packet loss
tc qdisc add dev eth0 root netem loss 10%

# Bandwidth limiting
tc qdisc add dev eth0 root netem rate 2mbit
```

### Validation Tests
1. **NAT Keepalive Survival**: Verify connection stays alive >2 minutes idle
2. **Timeout Recovery**: Test recovery from high latency scenarios
3. **Buffer Performance**: Test under burst conditions
4. **DNS Failure**: Test behavior when DNS is unavailable
5. **Peer Rebinding**: Verify logging and handling of Starlink port changes

## Backward Compatibility

- **Normal Internet**: All changes are transparent and improve performance
- **Existing Clients**: No breaking changes to the protocol
- **MAVLink Compatibility**: Keepalives never reach application layer
- **Configuration**: No additional configuration required

## Files Modified

- `droneengage_mavlink/src/udp_proxy/udpProxy.hpp` - Added timeout/keepalive declarations
- `droneengage_mavlink/src/udp_proxy/udpProxy.cpp` - Implemented Starlink hardening
- `droneengage_server/server/js_udp_proxy.js` - Added keepalive handling and logging

## Deployment Notes

1. **No Configuration Changes**: All improvements are automatic
2. **Logging**: Enhanced logging for diagnostics (peer changes, DNS resolution)
3. **Performance**: Minimal overhead (1-byte keepalive every 15s)
4. **Monitoring**: Watch for "UDP peer changed" logs to detect Starlink rebinding
