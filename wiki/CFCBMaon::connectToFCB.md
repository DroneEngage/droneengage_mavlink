`CFCBMain::connectToFCB` is a method that establishes a communication link to a flight control board (FCB) using MAVLink.

It determines the connection type—serial, UDP, or TCP—from a configuration file and initiates the appropriate connection via the `m_mavlink_sdk` interface.

---

### Definition

```cpp
95:135:/mnt/8a619ce7-cd3f-4520-af65-7991f16410f7/public_versions/drone_engage/drone_engage_mavlink/src/fcb_main.cpp
bool CFCBMain::connectToFCB() {
  m_connection_type = getConnectionType();

  switch (m_connection_type) {
  case CONNECTION_TYPE_SERIAL:
    // Read 'dynamic' flag from config
    // Log initialization
    m_mavlink_sdk.connectSerial(
        config["port"].get<std::string>().c_str(),
        config["baudrate"].get<int>(),
        dynamic);
    return true;

  case CONNECTION_TYPE_UDP:
    std::cout << "UDP Connection Initializing" << std::endl;
    m_mavlink_sdk.connectUDP(
        config["ip"].get<std::string>().c_str(),
        config["port"].get<int>());
    return true;

  case CONNECTION_TYPE_TCP:
    std::cout << "TCP Connection Initializing" << std::endl;
    m_mavlink_sdk.connectTCP(
        config["ip"].get<std::string>().c_str(),
        config["port"].get<int>());
    return true;

  default:
    throw "Connection to FCB is not in (serial, udp, tcp)";
  }

  return false;
}
```

- **Returns**: `bool` — Always returns `true` on valid connection types; throws an exception otherwise.
- **Side effects**: 
  - Initializes a connection via `m_mavlink_sdk` using serial, UDP, or TCP.
  - Outputs console messages indicating connection type and status.
  - May throw a string literal exception if the connection type is invalid.
- **Dependencies**: 
  - `m_jsonConfig`: JSON configuration containing connection parameters under `"fcb_connection_uri"`.
  - `getConnectionType()`: Infers connection type from the `"type"` field in config.
  - `m_mavlink_sdk`: External SDK instance responsible for actual MAVLink transport setup.

The method relies entirely on configuration data parsed earlier in `CFCBMain::init()` and assumes `m_jsonConfig` is already loaded.

---

### Example Usages

No direct callers are found in the codebase, suggesting `connectToFCB()` may be intended for future use, part of an incomplete integration, or called indirectly via base class interfaces not captured in static analysis.

Despite this, its design implies it should be invoked after configuration initialization and before MAVLink message exchange begins—likely during system startup in `CFCBMain::init()` or a derived initialization sequence.

Given the lack of usage traces, it appears to be **defined but currently unused** in the active code path.

---

### Notes

- The function **always returns `true`** for valid cases but includes a `return false;` after the `switch` block, which is unreachable due to the `throw` in `default`. This suggests a minor logic redundancy.
- It throws a **string literal** (`throw "Connection to FCB..."`) which is not type-safe; modern C++ practices recommend throwing objects derived from `std::exception`.
- The method assumes the existence and correctness of specific keys in `m_jsonConfig`, but only minimal validation is performed—highlighted by the `TODO` comment about adding checks for JSON fields.

---

### See Also

- `CFCBMain::getConnectionType()`: Determines the transport method from config; used directly by `connectToFCB` to decide connection flow.
- `m_jsonConfig["fcb_connection_uri"]`: Configuration object supplying connection parameters like `type`, `ip`, `port`, `baudrate`, and optional `dynamic` flag.
- `m_mavlink_sdk`: The underlying MAVLink SDK instance that performs actual transport-level connections; critical to the function’s operation.
- `CFCBMain::init()`: Initializes configuration and system state; likely intended precursor to calling `connectToFCB`.