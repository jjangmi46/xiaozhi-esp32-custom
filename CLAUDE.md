# Claude Code Context: Multi-Board Xiaozhi Robot System

## System Architecture Overview

This project involves THREE ESP32 boards working together as a robot system:

```
┌─────────────────────────────────────────────────────────────────┐
│                        PHONE HOTSPOT                             │
│                      (WiFi: bread_hot)                           │
└──────────────┬─────────────────────────────┬────────────────────┘
               │ WiFi                         │ WiFi
               ▼                              ▼
┌──────────────────────────┐    ┌──────────────────────────────────┐
│   FREENOVE ESP32-S3      │    │     XIAO ESP32-S3 SENSE          │
│   (Main Controller)      │    │     (Camera Node)                │
│                          │    │                                  │
│ - 2.8" LCD Display       │    │ - OV3660 Camera                  │
│ - Audio (I2S speaker/mic)│    │ - No display                     │
│ - Wake word detection    │    │ - No audio                       │
│ - MQTT/WebSocket to cloud│    │ - Cloud image upload             │
│ - Sends UART commands    │    │ - Receives UART commands         │
│                          │    │                                  │
│ Board: freenove-esp32s3- │    │ Board: xiao-esp32s3-camera-node  │
│        display-2.8-lcd   │    │                                  │
└───────────┬──────────────┘    └──────────────┬───────────────────┘
            │                                   │
            │ UART (GPIO2 TX → GPIO4 RX)       │
            │ UART (GPIO21 RX ← GPIO3 TX)      │
            └───────────────────────────────────┘
                              │
                              │ Shared 5V Power Rail
                              ▼
            ┌─────────────────────────────────────┐
            │       ESP32-C3 (Servo Controller)   │
            │                                     │
            │ - Controls 3 servo motors           │
            │ - Receives commands from Freenove   │
            │ - NO WiFi (uses shared power only)  │
            └─────────────────────────────────────┘
```

## Board Details

### 1. Freenove ESP32-S3 (Main Controller)
- **Location**: `main/boards/freenove-esp32s3-display-2.8-lcd/`
- **Role**: Main brain of the robot
- **Features**:
  - 2.8" LCD display for status/emotions
  - I2S audio codec (speaker + microphone)
  - Wake word detection (AFE)
  - Connects to xiaozhi cloud via MQTT/WebSocket
  - Sends UART commands to XIAO for camera operations
- **WiFi**: Connects to user's network for cloud services
- **UART**: TX=GPIO2, RX=GPIO21 (to XIAO)

### 2. XIAO ESP32-S3 Sense (Camera Node)
- **Location**: `main/boards/xiao-esp32s3-camera-node/`
- **Role**: Dedicated camera processing unit
- **Features**:
  - OV3660 camera sensor (DVP interface)
  - Captures images on UART command
  - Uploads directly to cloud vision API
  - Returns vision analysis results via UART
- **WiFi**: Connects to SAME network as Freenove for cloud upload
- **UART**: TX=GPIO3, RX=GPIO4 (to Freenove)
- **No audio, no display** - minimal resource usage

### 3. ESP32-C3 (Servo Controller)
- **Role**: Motor control
- **Features**:
  - Controls 3 servo motors
  - PWM output for servo positions
- **NO WiFi** - purely motor control
- **Power**: Shares 5V rail with other boards

## UART Protocol (Freenove ↔ XIAO)

Commands sent from Freenove to XIAO:

```
VISION:<url>|<token>|<device_id>|<client_id>
  - Configures the vision API endpoint and credentials
  - Response: "OK:Vision configured" or "ERR:<message>"

SNAP:<question>
  - Captures image and sends to cloud for analysis
  - <question> is the prompt for vision AI (e.g., "What do you see?")
  - Response: "OK:<vision_result>" or "ERR:<message>"
```

## Known WiFi Issues & Solutions

### Issue 1: `esp_wifi_get_mac failed with 12292`
- **Cause**: WifiConfigurationAp uses APSTA mode but only created AP netif
- **Fix**: Added STA netif creation in `wifi_configuration_ap.cc`
- **Files Modified**: `managed_components/78__esp-wifi-connect/wifi_configuration_ap.cc`

### Issue 2: WPA3 Authentication Timeout (0x200)
- **Cause**: Router using WPA3 which ESP32-S3 handles poorly
- **Fix**: Change router to WPA2 or WPA2/WPA3 mixed mode
- **Symptom**: `wifi:state: auth -> init (0x200)`

### Issue 3: Association Timeout (0x400)
- **Cause**: Power brownout during WiFi TX (servos drawing too much current)
- **Fix**: Disconnect servos during boot, or use separate power supplies
- **Symptom**: `wifi:state: assoc -> init (0x400)`

### Issue 4: Intermittent Connection (Both boards to same hotspot)
- **Cause**: Phone hotspot unreliability, power fluctuations
- **Current Status**: UNRESOLVED - needs investigation
- **Symptoms**:
  - Sometimes both connect successfully
  - Sometimes only one connects
  - Auth timeouts even with good signal (-23 to -45 dBm)
- **Attempted Fixes**:
  - Increased MAX_RECONNECT_COUNT from 5 to 10
  - Added better netif handling
  - Switched to WPA2

### Issue 5: AP Not Broadcasting
- **Cause**: Unknown - possibly related to STA netif creation
- **Status**: Intermittent - sometimes works, sometimes doesn't
- **Debug logging added** in StartAccessPoint()

## Power Configuration

**CRITICAL**: All boards share a single 5V power supply (3A max)

```
5V 3A Supply
     │
     ├──→ Freenove ESP32-S3 (~300-500mA with WiFi TX)
     ├──→ XIAO ESP32-S3 Sense (~300-500mA with WiFi TX + camera)
     ├──→ ESP32-C3 Servo Controller (~200mA)
     └──→ 3x Servo Motors (500mA-1A+ each when moving)
```

**Power Issues**:
- WiFi TX requires current spikes up to 500mA
- When servos move + both ESP32s TX simultaneously = brownout
- Brownout causes WiFi auth/assoc failures

**Recommendations**:
1. Use separate power for servos
2. Add large capacitors (1000µF+) on 5V rail
3. Stagger WiFi connections (don't boot both simultaneously)

## File Locations

### Modified Files (for WiFi fixes)
- `managed_components/78__esp-wifi-connect/wifi_configuration_ap.cc` - STA netif fix
- `managed_components/78__esp-wifi-connect/wifi_configuration_ap.h` - Added sta_netif_ member
- `managed_components/78__esp-wifi-connect/wifi_station.cc` - MAX_RECONNECT_COUNT

### Board Configurations
- `main/boards/xiao-esp32s3-camera-node/config.h` - XIAO pin definitions
- `main/boards/xiao-esp32s3-camera-node/xiao_camera_node.cc` - XIAO main code
- `main/boards/freenove-esp32s3-display-2.8-lcd/` - Freenove board code

### Camera Code
- `main/boards/common/esp32_camera.cc` - Camera driver (V4L2-based)
- `main/boards/common/esp32_camera.h` - Camera class definition

## Build Commands

```bash
# Build for Freenove
idf.py -B build_freenove set-target esp32s3
idf.py -B build_freenove menuconfig  # Select freenove board
idf.py -B build_freenove build flash monitor

# Build for XIAO Camera Node
idf.py -B build_xiao set-target esp32s3
idf.py -B build_xiao menuconfig  # Select xiao-esp32s3-camera-node
idf.py -B build_xiao build flash monitor
```

## Current Investigation Focus

The main unresolved issue is **intermittent WiFi connection when both boards try to connect to the same phone hotspot simultaneously**.

Hypotheses to test:
1. Phone hotspot has connection rate limiting
2. Both boards trying to connect causes RF interference
3. Power supply cannot handle both WiFi TXs simultaneously
4. Need to stagger connection attempts programmatically

## Contact / Session History

This documentation was created during a debugging session on 2026-02-10.
Main issues addressed:
- esp_wifi_get_mac error (FIXED)
- WPA3 compatibility (FIXED - user changed router)
- Power-related connection failures (PARTIALLY FIXED)
- Intermittent dual-board connection (IN PROGRESS)
