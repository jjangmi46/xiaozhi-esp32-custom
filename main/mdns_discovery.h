#pragma once

#include <string>
#include <cstdint>

/**
 * @brief mDNS service discovery for finding xiaozhi servers on local network.
 *
 * This module allows ESP32 devices to discover xiaozhi servers via mDNS
 * instead of relying on hardcoded IP addresses. This makes connections
 * stable even when the phone's hotspot IP changes.
 *
 * Service type: _xiaozhi._tcp
 * Service name: xiaozhi-server
 */
class MdnsDiscovery {
public:
    /**
     * @brief Result of mDNS discovery
     */
    struct DiscoveryResult {
        bool found;           ///< Whether a server was found
        std::string host;     ///< Server IP address (e.g., "192.168.43.1")
        uint16_t port;        ///< Server port (e.g., 8000)
        std::string url;      ///< Full OTA URL (e.g., "http://192.168.43.1:8000/xiaozhi/ota/")
    };

    /**
     * @brief Discover xiaozhi server on local network via mDNS
     *
     * This function queries for _xiaozhi._tcp services and returns
     * the first server found. It blocks until a server is found or
     * the timeout expires.
     *
     * @param timeout_ms Timeout in milliseconds (default: 5000ms)
     * @return DiscoveryResult with server information if found
     */
    static DiscoveryResult DiscoverServer(int timeout_ms = 5000);

    /**
     * @brief Initialize mDNS subsystem
     *
     * Must be called after WiFi is connected and before discovery.
     *
     * @return true if initialization succeeded
     */
    static bool Initialize();

    /**
     * @brief Deinitialize mDNS subsystem
     */
    static void Deinitialize();

private:
    static bool initialized_;
};
