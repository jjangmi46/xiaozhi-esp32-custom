#include "mdns_discovery.h"

#include <esp_log.h>
#include <mdns.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "MdnsDiscovery";

// Service type and name must match the Android app's mDNS registration
static const char* XIAOZHI_SERVICE_TYPE = "_xiaozhi";
static const char* XIAOZHI_SERVICE_PROTO = "_tcp";

bool MdnsDiscovery::initialized_ = false;

bool MdnsDiscovery::Initialize() {
    if (initialized_) {
        return true;
    }

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mDNS: %s", esp_err_to_name(err));
        return false;
    }

    // Set hostname for this device (optional, for debugging)
    mdns_hostname_set("xiaozhi-device");

    initialized_ = true;
    ESP_LOGI(TAG, "mDNS initialized successfully");
    return true;
}

void MdnsDiscovery::Deinitialize() {
    if (initialized_) {
        mdns_free();
        initialized_ = false;
        ESP_LOGI(TAG, "mDNS deinitialized");
    }
}

MdnsDiscovery::DiscoveryResult MdnsDiscovery::DiscoverServer(int timeout_ms) {
    DiscoveryResult result = {false, "", 0, ""};

    if (!initialized_) {
        ESP_LOGW(TAG, "mDNS not initialized, initializing now...");
        if (!Initialize()) {
            return result;
        }
    }

    ESP_LOGI(TAG, "Searching for %s.%s services (timeout: %dms)...",
             XIAOZHI_SERVICE_TYPE, XIAOZHI_SERVICE_PROTO, timeout_ms);

    // Query for _xiaozhi._tcp services
    mdns_result_t* results = nullptr;
    esp_err_t err = mdns_query_ptr(XIAOZHI_SERVICE_TYPE, XIAOZHI_SERVICE_PROTO,
                                    timeout_ms, 10, &results);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mDNS query failed: %s", esp_err_to_name(err));
        return result;
    }

    if (results == nullptr) {
        ESP_LOGW(TAG, "No xiaozhi servers found via mDNS");
        return result;
    }

    // Process the first result
    mdns_result_t* r = results;
    while (r != nullptr) {
        ESP_LOGI(TAG, "Found service: %s", r->instance_name ? r->instance_name : "(no name)");

        // Try to get the IP address
        if (r->addr != nullptr) {
            // Found IPv4 address
            char ip_str[16];
            if (r->addr->addr.type == ESP_IPADDR_TYPE_V4) {
                snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&r->addr->addr.u_addr.ip4));

                result.found = true;
                result.host = ip_str;
                result.port = r->port;

                // Build the OTA URL
                char url_buf[128];
                snprintf(url_buf, sizeof(url_buf), "http://%s:%d/xiaozhi/ota/",
                         ip_str, r->port);
                result.url = url_buf;

                ESP_LOGI(TAG, "Discovered server at %s:%d", ip_str, r->port);
                ESP_LOGI(TAG, "OTA URL: %s", result.url.c_str());

                // Check TXT records for additional info
                mdns_txt_item_t* txt = r->txt;
                for (int i = 0; i < r->txt_count; i++) {
                    ESP_LOGD(TAG, "  TXT: %s=%s", txt[i].key, txt[i].value ? txt[i].value : "");
                }

                break;  // Use the first valid result
            }
        } else if (r->hostname != nullptr) {
            // No direct IP, try to resolve hostname
            ESP_LOGI(TAG, "Service hostname: %s, port: %d", r->hostname, r->port);

            // Query for the A record of the hostname
            esp_ip4_addr_t addr;
            err = mdns_query_a(r->hostname, timeout_ms, &addr);
            if (err == ESP_OK) {
                char ip_str[16];
                snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&addr));

                result.found = true;
                result.host = ip_str;
                result.port = r->port;

                char url_buf[128];
                snprintf(url_buf, sizeof(url_buf), "http://%s:%d/xiaozhi/ota/",
                         ip_str, r->port);
                result.url = url_buf;

                ESP_LOGI(TAG, "Resolved %s to %s:%d", r->hostname, ip_str, r->port);
                break;
            } else {
                ESP_LOGW(TAG, "Failed to resolve hostname %s: %s",
                         r->hostname, esp_err_to_name(err));
            }
        }

        r = r->next;
    }

    // Free the results
    mdns_query_results_free(results);

    if (!result.found) {
        ESP_LOGW(TAG, "Could not resolve any discovered server addresses");
    }

    return result;
}
