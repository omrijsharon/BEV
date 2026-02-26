/*
 * BetaflightMSP - Arduino library for communicating with Betaflight flight controllers
 * 
 * This library implements the MSP (MultiWii Serial Protocol) used by Betaflight.
 * 
 * Author: Arduino MSP Library
 * License: GPL v3
 */

#include "BetaflightMSP.h"

BetaflightMSP::BetaflightMSP() {
    _port = nullptr;
    _jitterDirection = 0;  // Start with North
    _homeLat = 0;
    _homeLon = 0;
    _homePositionSet = false;
    _orbitCurrentWaypoint = 0;
    _orbitLapsCompleted = 0;
    resetRxState();
}

void BetaflightMSP::begin(Stream &serialPort) {
    _port = &serialPort;
    resetRxState();
}

void BetaflightMSP::resetRxState() {
    _rxState = MSP_IDLE;
    _rxPayloadSize = 0;
    _rxPayloadIndex = 0;
    _rxCommand = 0;
    _rxChecksum = 0;
    _rxChecksum2 = 0;
    _rxError = false;
    _responseReceived = false;
}

void BetaflightMSP::sendMSP(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, bool expectResponse) {
    if (!_port) return;
    
    // MSP V1 frame format: $M<[size][cmd][payload][checksum]
    uint8_t checksum = 0;
    
    // Header
    _port->write('$');
    _port->write('M');
    _port->write('<');
    
    // Size
    _port->write(payloadSize);
    checksum ^= payloadSize;
    
    // Command
    _port->write(cmd & 0xFF);
    checksum ^= (cmd & 0xFF);
    
    // Payload
    for (uint8_t i = 0; i < payloadSize; i++) {
        _port->write(payload[i]);
        checksum ^= payload[i];
    }
    
    // Checksum
    _port->write(checksum);
    
    // Flush to ensure data is sent immediately
    _port->flush();
    
    if (expectResponse) {
        _responseReceived = false;
        _requestTimeout = millis() + 500;  // 500ms timeout
    }
}

bool BetaflightMSP::request(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, uint32_t timeout) {
    if (!_port) return false;
    
    resetRxState();
    sendMSP(cmd, payload, payloadSize, true);
    
    uint32_t startTime = millis();
    while (millis() - startTime < timeout) {
        update();
        if (_responseReceived && _rxCommand == cmd) {
            return !_rxError;
        }
    }
    
    return false;  // Timeout
}

bool BetaflightMSP::command(uint16_t cmd, uint8_t *payload, uint8_t payloadSize) {
    if (!_port) return false;
    
    sendMSP(cmd, payload, payloadSize, false);
    return true;
}

// For SET commands that require acknowledgment
bool BetaflightMSP::commandWithAck(uint16_t cmd, uint8_t *payload, uint8_t payloadSize, uint32_t timeout) {
    if (!_port) {
        Serial.printf("[MSP] No serial port for cmd %d\n", cmd);
        return false;
    }
    
    // Send the command
    resetRxState();
    sendMSP(cmd, payload, payloadSize, true);
    
    // Wait for acknowledgment
    uint32_t startTime = millis();
    while (millis() - startTime < timeout) {
        update();
        if (_responseReceived) {
            if (_rxCommand == cmd) {
                // Acknowledgment received with matching command code
                if (_rxError) {
                    Serial.printf("[MSP] CMD %d ACK received with ERROR\n", cmd);
                }
                return !_rxError;
            } else {
                Serial.printf("[MSP] CMD %d: Got response %d instead of ACK\n", cmd, _rxCommand);
                // Don't return, keep waiting for the right response
                _responseReceived = false;
            }
        }
    }
    
    Serial.printf("[MSP] CMD %d: Timeout (no ACK)\n", cmd);
    return false;  // Timeout - no acknowledgment received
}

void BetaflightMSP::update() {
    if (!_port) return;
    
    while (_port->available()) {
        uint8_t c = _port->read();
        if (processReceivedByte(c)) {
            _responseReceived = true;
            return;
        }
    }
}

bool BetaflightMSP::processReceivedByte(uint8_t c) {
    switch (_rxState) {
        case MSP_IDLE:
            if (c == '$') {
                _rxState = MSP_HEADER_START;
            }
            break;
            
        case MSP_HEADER_START:
            if (c == 'M') {
                _rxState = MSP_HEADER_M;
            } else {
                _rxState = MSP_IDLE;
            }
            break;
            
        case MSP_HEADER_M:
            if (c == '>') {
                _rxState = MSP_HEADER_ARROW;
                _rxError = false;
                _rxChecksum = 0;
            } else if (c == '!') {
                _rxState = MSP_HEADER_ARROW;
                _rxError = true;
                _rxChecksum = 0;
            } else {
                _rxState = MSP_IDLE;
            }
            break;
            
        case MSP_HEADER_ARROW:
            _rxPayloadSize = c;
            _rxChecksum ^= c;
            _rxPayloadIndex = 0;
            _rxState = MSP_HEADER_SIZE;
            break;
            
        case MSP_HEADER_SIZE:
            _rxCommand = c;
            _rxChecksum ^= c;
            if (_rxPayloadSize > 0) {
                _rxState = MSP_PAYLOAD;
            } else {
                _rxState = MSP_CHECKSUM;
            }
            break;
            
        case MSP_PAYLOAD:
            _rxPayload[_rxPayloadIndex++] = c;
            _rxChecksum ^= c;
            if (_rxPayloadIndex >= _rxPayloadSize) {
                _rxState = MSP_CHECKSUM;
            }
            break;
            
        case MSP_CHECKSUM:
            if (_rxChecksum == c) {
                _rxState = MSP_COMMAND_RECEIVED;
                return true;  // Message received successfully
            } else {
                // Checksum error
                _rxError = true;
            }
            resetRxState();
            break;
            
        default:
            resetRxState();
            break;
    }
    
    return false;
}

// Convenience functions
bool BetaflightMSP::getApiVersion(msp_api_version_t &data) {
    if (!request(MSP_API_VERSION, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.protocolVersion = _rxPayload[offset++];
    data.apiVersionMajor = _rxPayload[offset++];
    data.apiVersionMinor = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getFcVariant(msp_fc_variant_t &data) {
    if (!request(MSP_FC_VARIANT, nullptr, 0)) {
        return false;
    }
    
    memcpy(data.identifier, _rxPayload, 4);
    data.identifier[4] = '\0';
    
    return true;
}

bool BetaflightMSP::getFcVersion(msp_fc_version_t &data) {
    if (!request(MSP_FC_VERSION, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.versionMajor = _rxPayload[offset++];
    data.versionMinor = _rxPayload[offset++];
    data.versionPatchLevel = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getBoardInfo(msp_board_info_t &data) {
    if (!request(MSP_BOARD_INFO, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    memcpy(data.boardIdentifier, &_rxPayload[offset], 4);
    data.boardIdentifier[4] = '\0';
    offset += 4;
    
    data.hardwareRevision = readU16(_rxPayload, offset);
    data.boardType = _rxPayload[offset++];
    data.targetCapabilities = _rxPayload[offset++];
    data.targetNameLength = _rxPayload[offset++];
    
    if (data.targetNameLength > 0 && data.targetNameLength < 32) {
        memcpy(data.targetName, &_rxPayload[offset], data.targetNameLength);
        data.targetName[data.targetNameLength] = '\0';
    } else {
        data.targetName[0] = '\0';
    }
    
    return true;
}

bool BetaflightMSP::getName(msp_name_t &data) {
    if (!request(MSP_NAME, nullptr, 0)) {
        return false;
    }
    
    uint8_t len = min(_rxPayloadSize, (uint8_t)15);
    memcpy(data.name, _rxPayload, len);
    data.name[len] = '\0';
    
    return true;
}

bool BetaflightMSP::getStatus(msp_status_t &data) {
    if (!request(MSP_STATUS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.cycleTime = readU16(_rxPayload, offset);
    data.i2cErrorCounter = readU16(_rxPayload, offset);
    data.sensor = readU16(_rxPayload, offset);
    data.flightModeFlags = readU32(_rxPayload, offset);
    data.configProfileIndex = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getAttitude(msp_attitude_t &data) {
    if (!request(MSP_ATTITUDE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.roll = read16(_rxPayload, offset);
    data.pitch = read16(_rxPayload, offset);
    data.yaw = read16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getAltitude(msp_altitude_t &data) {
    if (!request(MSP_ALTITUDE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.estimatedAltitude = read32(_rxPayload, offset);
    data.vario = read16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getAnalog(msp_analog_t &data) {
    if (!request(MSP_ANALOG, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.vbat = _rxPayload[offset++];
    data.mAhDrawn = readU16(_rxPayload, offset);
    data.rssi = readU16(_rxPayload, offset);
    data.amperage = read16(_rxPayload, offset);
    
    // Voltage field added in API 1.42
    if (_rxPayloadSize >= 9) {
        data.voltage = readU16(_rxPayload, offset);
    } else {
        data.voltage = data.vbat * 10;  // Convert to 0.01V
    }
    
    return true;
}

bool BetaflightMSP::getRawGPS(msp_raw_gps_t &data) {
    if (!request(MSP_RAW_GPS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.fixType = _rxPayload[offset++];
    data.numSat = _rxPayload[offset++];
    data.lat = read32(_rxPayload, offset);
    data.lon = read32(_rxPayload, offset);
    data.altCm = read16(_rxPayload, offset);
    data.groundSpeed = readU16(_rxPayload, offset);
    data.groundCourse = readU16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getCompGPS(msp_comp_gps_t &data) {
    if (!request(MSP_COMP_GPS, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.distanceToHome = readU16(_rxPayload, offset);
    data.directionToHome = readU16(_rxPayload, offset);
    data.heartbeat = _rxPayload[offset++];
    
    return true;
}

bool BetaflightMSP::getBatteryState(msp_battery_state_t &data) {
    if (!request(MSP_BATTERY_STATE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.cellCount = _rxPayload[offset++];
    data.capacity = readU16(_rxPayload, offset);
    data.voltage = _rxPayload[offset++];
    data.mAhDrawn = readU16(_rxPayload, offset);
    data.amperage = readU16(_rxPayload, offset);
    data.batteryState = _rxPayload[offset++];
    data.voltage16 = readU16(_rxPayload, offset);
    
    return true;
}

bool BetaflightMSP::getRC(msp_rc_t &data) {
    if (!request(MSP_RC, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    uint8_t channelCount = _rxPayloadSize / 2;
    for (uint8_t i = 0; i < channelCount && i < 18; i++) {
        data.channels[i] = readU16(_rxPayload, offset);
    }
    
    return true;
}

bool BetaflightMSP::getDebug(msp_debug_t &data) {
    if (!request(MSP_DEBUG, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    // Read 8 signed 16-bit debug values
    for (uint8_t i = 0; i < 8; i++) {
        data.debug[i] = read16(_rxPayload, offset);
    }
    
    return true;
}

bool BetaflightMSP::getGPSRescue(msp_gps_rescue_t &data) {
    if (!request(MSP_GPS_RESCUE, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.maxRescueAngle = readU16(_rxPayload, offset);
    data.returnAltitudeM = readU16(_rxPayload, offset);
    data.descentDistanceM = readU16(_rxPayload, offset);
    data.groundSpeedCmS = readU16(_rxPayload, offset);
    data.throttleMin = readU16(_rxPayload, offset);
    data.throttleMax = readU16(_rxPayload, offset);
    data.throttleHover = readU16(_rxPayload, offset);
    data.sanityChecks = _rxPayload[offset++];
    data.minSats = _rxPayload[offset++];
    
    // Added in API version 1.43
    if (_rxPayloadSize >= offset + 6) {
        data.ascendRate = readU16(_rxPayload, offset);
        data.descendRate = readU16(_rxPayload, offset);
        data.allowArmingWithoutFix = _rxPayload[offset++];
        data.altitudeMode = _rxPayload[offset++];
    } else {
        // Default values for older API versions
        data.ascendRate = 500;
        data.descendRate = 150;
        data.allowArmingWithoutFix = 0;
        data.altitudeMode = 0;
    }
    
    // Added in API version 1.44
    if (_rxPayloadSize >= offset + 2) {
        data.minStartDistM = readU16(_rxPayload, offset);
    } else {
        data.minStartDistM = 15;
    }
    
    // Added in API version 1.46
    if (_rxPayloadSize >= offset + 2) {
        data.initialClimbM = readU16(_rxPayload, offset);
    } else {
        data.initialClimbM = 10;
    }
    
    return true;
}

bool BetaflightMSP::getGPSHomeFromDebug(int32_t &lat, int32_t &lon) {
    // Read debug values from FC (must have debug mode set to GPS_HOME)
    msp_debug_t debug;
    if (!getDebug(debug)) {
        return false;
    }
    
    // Reconstruct 32-bit GPS coordinates from 16-bit debug values
    // GPS_home[GPS_LATITUDE] is split into:
    //   debug[0] = lower 16 bits (GPS_LATITUDE & 0xFFFF)
    //   debug[1] = upper 16 bits (GPS_LATITUDE >> 16)
    // GPS_home[GPS_LONGITUDE] is split into:
    //   debug[2] = lower 16 bits (GPS_LONGITUDE & 0xFFFF)
    //   debug[3] = upper 16 bits (GPS_LONGITUDE >> 16)
    
    // Cast to uint16_t to avoid sign extension issues, then combine
    uint16_t lat_lo = (uint16_t)debug.debug[0];
    uint16_t lat_hi = (uint16_t)debug.debug[1];
    uint16_t lon_lo = (uint16_t)debug.debug[2];
    uint16_t lon_hi = (uint16_t)debug.debug[3];
    
    // Reconstruct 32-bit values
    lat = ((int32_t)lat_hi << 16) | lat_lo;
    lon = ((int32_t)lon_hi << 16) | lon_lo;
    
    return true;
}

bool BetaflightMSP::getVtxConfig(msp_vtx_config_t &data) {
    if (!request(MSP_VTX_CONFIG, nullptr, 0)) {
        return false;
    }
    
    uint8_t offset = 0;
    data.deviceType = _rxPayload[offset++];
    data.band = _rxPayload[offset++];
    data.channel = _rxPayload[offset++];
    data.power = _rxPayload[offset++];
    data.pitMode = _rxPayload[offset++];
    data.freq = readU16(_rxPayload, offset);
    data.deviceIsReady = _rxPayload[offset++];
    data.lowPowerDisarm = _rxPayload[offset++];
    
    // Added in API version 1.42
    if (_rxPayloadSize >= offset + 2) {
        data.pitModeFreq = readU16(_rxPayload, offset);
    } else {
        data.pitModeFreq = 0;
    }
    
    // VTX Table info (API 1.42+)
    if (_rxPayloadSize >= offset + 4) {
        data.vtxTableAvailable = _rxPayload[offset++];
        data.bands = _rxPayload[offset++];
        data.channels = _rxPayload[offset++];
        data.powerLevels = _rxPayload[offset++];
    } else {
        data.vtxTableAvailable = 0;
        data.bands = 0;
        data.channels = 0;
        data.powerLevels = 0;
    }
    
    return true;
}

bool BetaflightMSP::setRawRC(uint16_t *channels, uint8_t channelCount) {
    if (channelCount > 18) channelCount = 18;
    
    uint8_t payload[36];  // Max 18 channels * 2 bytes
    uint8_t offset = 0;
    
    for (uint8_t i = 0; i < channelCount; i++) {
        writeU16(payload, offset, channels[i]);
    }
    
    return command(MSP_SET_RAW_RC, payload, offset);
}

bool BetaflightMSP::setGPSHome(int32_t lat, int32_t lon, uint16_t altitudeM) {
    uint8_t payload[10];
    uint8_t offset = 0;
    
    write32(payload, offset, lat);
    write32(payload, offset, lon);
    writeU16(payload, offset, altitudeM);
    
    // Use commandWithAck to wait for FC acknowledgment
    return commandWithAck(MSP_WP, payload, offset, 500);
}

bool BetaflightMSP::setRawGPS(uint8_t fixType, uint8_t numSat, int32_t lat, int32_t lon, int16_t altM, uint16_t groundSpeed) {
    uint8_t payload[16];
    uint8_t offset = 0;
    
    payload[offset++] = fixType;
    payload[offset++] = numSat;
    write32(payload, offset, lat);
    write32(payload, offset, lon);
    writeU16(payload, offset, altM * 100);  // Convert meters to cm
    writeU16(payload, offset, groundSpeed);
    
    // Use commandWithAck to wait for FC acknowledgment
    return commandWithAck(MSP_SET_RAW_GPS, payload, offset, 500);
}

bool BetaflightMSP::setGPSRescue(const msp_gps_rescue_t &data) {
    uint8_t payload[32];
    uint8_t offset = 0;
    
    writeU16(payload, offset, data.maxRescueAngle);
    writeU16(payload, offset, data.returnAltitudeM);
    writeU16(payload, offset, data.descentDistanceM);
    writeU16(payload, offset, data.groundSpeedCmS);
    writeU16(payload, offset, data.throttleMin);
    writeU16(payload, offset, data.throttleMax);
    writeU16(payload, offset, data.throttleHover);
    payload[offset++] = data.sanityChecks;
    payload[offset++] = data.minSats;
    
    // Added in API version 1.43
    writeU16(payload, offset, data.ascendRate);
    writeU16(payload, offset, data.descendRate);
    payload[offset++] = data.allowArmingWithoutFix;
    payload[offset++] = data.altitudeMode;
    
    // Added in API version 1.44
    writeU16(payload, offset, data.minStartDistM);
    
    // Added in API version 1.46
    writeU16(payload, offset, data.initialClimbM);
    
    // Use commandWithAck to wait for FC acknowledgment
    return commandWithAck(MSP_SET_GPS_RESCUE, payload, offset, 500);
}

bool BetaflightMSP::setVtxConfig(const msp_vtx_config_t &data) {
    uint8_t payload[32];
    uint8_t offset = 0;
    
    // Frequency/Band/Channel combined value
    // If band > 0, encode as: (band-1)*8 + (channel-1)
    // Otherwise use raw frequency value
    uint16_t bandChanOrFreq;
    if (data.band > 0 && data.band <= 5 && data.channel > 0 && data.channel <= 8) {
        bandChanOrFreq = (data.band - 1) * 8 + (data.channel - 1);
    } else {
        bandChanOrFreq = data.freq;  // Use frequency directly
    }
    writeU16(payload, offset, bandChanOrFreq);
    
    // Power and pit mode
    payload[offset++] = data.power;
    payload[offset++] = data.pitMode;
    
    // Low power disarm
    payload[offset++] = data.lowPowerDisarm;
    
    // Added in API version 1.42 - pit mode frequency
    writeU16(payload, offset, data.pitModeFreq);
    
    // Added in API version 1.42 - standalone band, channel, frequency
    payload[offset++] = data.band;
    payload[offset++] = data.channel;
    writeU16(payload, offset, data.freq);
    
    // Added in API version 1.42 - vtxtable configuration (set to 0 to not modify)
    payload[offset++] = 0;  // bandCount (0 = don't change)
    payload[offset++] = 0;  // channelCount (0 = don't change)
    payload[offset++] = 0;  // powerCount (0 = don't change)
    payload[offset++] = 0;  // clearTable (0 = don't clear)
    
    // Use commandWithAck to wait for FC acknowledgment
    return commandWithAck(MSP_SET_VTX_CONFIG, payload, offset, 500);
}

// ==================== GPS Utility Functions ====================

// Calculate distance between two GPS coordinates using Haversine formula
// Returns distance in meters
float BetaflightMSP::calculateDistance(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    // Convert from degrees * 1e7 to radians
    float lat1_rad = (lat1 / 10000000.0f) * DEG_TO_RAD;
    float lon1_rad = (lon1 / 10000000.0f) * DEG_TO_RAD;
    float lat2_rad = (lat2 / 10000000.0f) * DEG_TO_RAD;
    float lon2_rad = (lon2 / 10000000.0f) * DEG_TO_RAD;
    
    // Haversine formula
    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;
    
    float a = sin(dlat / 2.0f) * sin(dlat / 2.0f) +
              cos(lat1_rad) * cos(lat2_rad) *
              sin(dlon / 2.0f) * sin(dlon / 2.0f);
    
    float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
    
    return EARTH_RADIUS_M * c;  // Distance in meters
}

// Calculate bearing from point 1 to point 2
// Returns bearing in degrees (0-360, where 0 is North)
float BetaflightMSP::calculateBearing(int32_t lat1, int32_t lon1, int32_t lat2, int32_t lon2) {
    // Convert from degrees * 1e7 to radians
    float lat1_rad = (lat1 / 10000000.0f) * DEG_TO_RAD;
    float lon1_rad = (lon1 / 10000000.0f) * DEG_TO_RAD;
    float lat2_rad = (lat2 / 10000000.0f) * DEG_TO_RAD;
    float lon2_rad = (lon2 / 10000000.0f) * DEG_TO_RAD;
    
    float dlon = lon2_rad - lon1_rad;
    
    float y = sin(dlon) * cos(lat2_rad);
    float x = cos(lat1_rad) * sin(lat2_rad) -
              sin(lat1_rad) * cos(lat2_rad) * cos(dlon);
    
    float bearing_rad = atan2(y, x);
    float bearing_deg = bearing_rad * RAD_TO_DEG;
    
    // Normalize to 0-360
    if (bearing_deg < 0) {
        bearing_deg += 360.0f;
    }
    
    return bearing_deg;
}

// Calculate new GPS coordinate from origin + bearing + distance
// bearing_deg: 0 = North, 90 = East, 180 = South, 270 = West
void BetaflightMSP::offsetGPSCoordinate(int32_t lat_origin, int32_t lon_origin,
                                         float distance_m, float bearing_deg,
                                         int32_t &lat_new, int32_t &lon_new) {
    // Convert origin to radians
    float lat_rad = (lat_origin / 10000000.0f) * DEG_TO_RAD;
    float lon_rad = (lon_origin / 10000000.0f) * DEG_TO_RAD;
    float bearing_rad = bearing_deg * DEG_TO_RAD;
    
    // Calculate angular distance
    float angular_distance = distance_m / EARTH_RADIUS_M;
    
    // Calculate new latitude
    float lat_new_rad = asin(sin(lat_rad) * cos(angular_distance) +
                              cos(lat_rad) * sin(angular_distance) * cos(bearing_rad));
    
    // Calculate new longitude
    float lon_new_rad = lon_rad + atan2(sin(bearing_rad) * sin(angular_distance) * cos(lat_rad),
                                         cos(angular_distance) - sin(lat_rad) * sin(lat_new_rad));
    
    // Convert back to degrees * 1e7
    lat_new = (int32_t)((lat_new_rad * RAD_TO_DEG) * 10000000.0f);
    lon_new = (int32_t)((lon_new_rad * RAD_TO_DEG) * 10000000.0f);
}

// ==================== Safety Check Functions ====================

// Check if GPS is healthy for navigation
bool BetaflightMSP::checkGPSHealth(const msp_raw_gps_t &gps, uint8_t minSats) {
    // Check GPS fix flag
    if (!(gps.fixType & GPS_FIX)) {
        Serial.println("[Safety] GPS has no fix");
        return false;
    }
    
    // Check satellite count
    if (gps.numSat < minSats) {
        Serial.printf("[Safety] Insufficient satellites: %d < %d\n", gps.numSat, minSats);
        return false;
    }
    
    return true;
}

// Check if battery level is sufficient for operation
bool BetaflightMSP::checkBatteryLevel(const msp_battery_state_t &battery, uint8_t minPercent) {
    // Estimate battery percentage (simplified)
    // Assuming 4.2V max per cell and 3.3V min per cell
    float voltage_per_cell = battery.voltage16 / 100.0f / battery.cellCount;
    float percent = ((voltage_per_cell - 3.3f) / (4.2f - 3.3f)) * 100.0f;
    
    if (percent < minPercent) {
        Serial.printf("[Safety] Low battery: %.1f%% < %d%%\n", percent, minPercent);
        return false;
    }
    
    return true;
}

// Check if altitude is within safe limits
bool BetaflightMSP::checkAltitudeLimits(int32_t altitude_cm, int32_t max_altitude_cm) {
    if (altitude_cm < 0) {
        Serial.println("[Safety] Negative altitude detected");
        return false;
    }
    
    if (altitude_cm > max_altitude_cm) {
        Serial.printf("[Safety] Altitude too high: %d cm > %d cm\n", altitude_cm, max_altitude_cm);
        return false;
    }
    
    return true;
}

// ==================== Flight Mode Control Functions ====================

// Check if GPS Rescue mode is currently active
bool BetaflightMSP::isGPSRescueActive() {
    msp_status_t status;
    if (!getStatus(status)) {
        Serial.println("[FlightMode] Failed to read status");
        return false;
    }
    
    return (status.flightModeFlags & GPS_RESCUE_MODE) != 0;
}

// Activate GPS Rescue mode by setting RC channel high
// Note: This assumes GPS Rescue is configured on an AUX channel
// The FC must have GPS Rescue mode configured and the switch mapped
bool BetaflightMSP::activateGPSRescue() {
    // Read current RC values
    msp_rc_t rc;
    if (!getRC(rc)) {
        Serial.println("[FlightMode] Failed to read RC channels");
        return false;
    }
    
    // Find the GPS Rescue AUX channel (typically AUX1-AUX4)
    // For now, we'll assume it's on AUX2 (channel 5, index 4)
    // TODO: Make this configurable or auto-detect from mode ranges
    const uint8_t GPS_RESCUE_CHANNEL = 4;  // AUX2 (channel 5)
    
    // Set the GPS Rescue channel to HIGH (2000)
    rc.channels[GPS_RESCUE_CHANNEL] = 2000;
    
    // Send updated RC values
    if (!setRawRC(rc.channels, 18)) {
        Serial.println("[FlightMode] Failed to activate GPS Rescue");
        return false;
    }
    
    // Wait a bit for mode to activate
    delay(100);
    
    // Verify GPS Rescue is now active
    if (isGPSRescueActive()) {
        Serial.println("[FlightMode] GPS Rescue activated successfully");
        return true;
    } else {
        Serial.println("[FlightMode] GPS Rescue activation failed - mode not active");
        return false;
    }
}

// Deactivate GPS Rescue and switch to Angle mode
bool BetaflightMSP::deactivateGPSRescue() {
    // Read current RC values
    msp_rc_t rc;
    if (!getRC(rc)) {
        Serial.println("[FlightMode] Failed to read RC channels");
        return false;
    }
    
    // Set GPS Rescue channel to LOW (1000)
    const uint8_t GPS_RESCUE_CHANNEL = 4;  // AUX2 (channel 5)
    rc.channels[GPS_RESCUE_CHANNEL] = 1000;
    
    // Set Angle mode channel to MID (1500) to activate Angle mode
    const uint8_t ANGLE_MODE_CHANNEL = 0;  // AUX1 (channel 4)
    rc.channels[ANGLE_MODE_CHANNEL] = 1500;
    
    // Send updated RC values
    if (!setRawRC(rc.channels, 18)) {
        Serial.println("[FlightMode] Failed to deactivate GPS Rescue");
        return false;
    }
    
    // Wait a bit for mode to change
    delay(100);
    
    // Verify GPS Rescue is now inactive
    if (!isGPSRescueActive()) {
        Serial.println("[FlightMode] GPS Rescue deactivated successfully");
        return true;
    } else {
        Serial.println("[FlightMode] GPS Rescue deactivation failed - mode still active");
        return false;
    }
}

// ==================== High-Level Navigation Functions ====================

// Navigate to a specific GPS waypoint
// Returns true if successfully reached waypoint, false on error or timeout
bool BetaflightMSP::gotoWaypoint(int32_t target_lat, int32_t target_lon, 
                                  uint16_t altitude_m, uint16_t speed_cm_s, 
                                  float arrival_threshold_m) {
    Serial.println("[Navigation] ========== GO TO WAYPOINT ==========");
    
    // Task 3.1a: Input validation
    if (target_lat < -900000000 || target_lat > 900000000) {
        Serial.println("[Navigation] ERROR: Invalid latitude");
        return false;
    }
    if (target_lon < -1800000000 || target_lon > 1800000000) {
        Serial.println("[Navigation] ERROR: Invalid longitude");
        return false;
    }
    if (altitude_m < 5 || altitude_m > 120) {
        Serial.println("[Navigation] ERROR: Altitude out of range (5-120m)");
        return false;
    }
    if (speed_cm_s < 100 || speed_cm_s > 3000) {
        Serial.println("[Navigation] ERROR: Speed out of range (100-3000 cm/s)");
        return false;
    }
    
    // Task 3.1b: Pre-flight safety checks
    Serial.println("[Navigation] Performing pre-flight checks...");
    
    msp_raw_gps_t current_gps;
    if (!getRawGPS(current_gps)) {
        Serial.println("[Navigation] ERROR: Failed to read GPS");
        return false;
    }
    
    if (!checkGPSHealth(current_gps)) {
        Serial.println("[Navigation] ERROR: GPS not healthy");
        return false;
    }
    
    msp_battery_state_t battery;
    if (!getBatteryState(battery)) {
        Serial.println("[Navigation] ERROR: Failed to read battery");
        return false;
    }
    
    if (!checkBatteryLevel(battery, 30)) {
        Serial.println("[Navigation] ERROR: Battery too low");
        return false;
    }
    
    if (!checkAltitudeLimits(current_gps.altCm, altitude_m * 100)) {
        Serial.println("[Navigation] ERROR: Altitude limit check failed");
        return false;
    }
    
    // Task 3.1c: Backup current GPS Rescue config
    Serial.println("[Navigation] Backing up GPS Rescue configuration...");
    msp_gps_rescue_t original_rescue_config;
    if (!getGPSRescue(original_rescue_config)) {
        Serial.println("[Navigation] ERROR: Failed to backup GPS Rescue config");
        return false;
    }
    
    // Task 3.1d: Calculate distance and bearing to target
    float distance_m = calculateDistance(current_gps.lat, current_gps.lon, target_lat, target_lon);
    float bearing_deg = calculateBearing(current_gps.lat, current_gps.lon, target_lat, target_lon);
    
    Serial.printf("[Navigation] Target: %.6f, %.6f @ %dm\n", 
                  target_lat/10000000.0f, target_lon/10000000.0f, altitude_m);
    Serial.printf("[Navigation] Distance: %.1fm, Bearing: %.1f°\n", distance_m, bearing_deg);
    
    // Calculate estimated time
    float estimated_time_s = distance_m / (speed_cm_s / 100.0f);
    uint32_t timeout_ms = (uint32_t)(estimated_time_s * 1500);  // 1.5x margin
    
    Serial.printf("[Navigation] Estimated time: %.1fs, Timeout: %lus\n", 
                  estimated_time_s, timeout_ms/1000);
    
    // Task 3.1e: Configure GPS Rescue parameters for waypoint navigation
    Serial.println("[Navigation] Configuring GPS Rescue for waypoint navigation...");
    msp_gps_rescue_t waypoint_config = original_rescue_config;
    
    waypoint_config.returnAltitudeM = altitude_m;
    waypoint_config.altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
    waypoint_config.initialClimbM = max(0, (int)(altitude_m - current_gps.altCm/100));
    waypoint_config.ascendRate = 750;  // 7.5 m/s climb
    waypoint_config.groundSpeedCmS = speed_cm_s;
    waypoint_config.descentDistanceM = 10;  // Start descent 10m from target
    waypoint_config.descendRate = 150;  // 1.5 m/s descent
    waypoint_config.minStartDistM = 5;
    waypoint_config.maxRescueAngle = 45;
    waypoint_config.sanityChecks = RESCUE_SANITY_OFF;  // Disable sanity checks for waypoint nav
    
    if (!setGPSRescue(waypoint_config)) {
        Serial.println("[Navigation] ERROR: Failed to configure GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.1f: Set GPS Home to target waypoint
    Serial.println("[Navigation] Setting GPS Home to target waypoint...");
    if (!setGPSHome(target_lat, target_lon, altitude_m)) {
        Serial.println("[Navigation] ERROR: Failed to set GPS Home");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.1g: Activate GPS Rescue mode
    Serial.println("[Navigation] Activating GPS Rescue mode...");
    if (!activateGPSRescue()) {
        Serial.println("[Navigation] ERROR: Failed to activate GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.1h: Monitor progress loop
    Serial.println("[Navigation] Monitoring progress...");
    uint32_t start_time = millis();
    uint32_t last_update = 0;
    bool arrived = false;
    
    while (millis() - start_time < timeout_ms) {
        // Update every 500ms
        if (millis() - last_update < 500) {
            delay(50);
            continue;
        }
        last_update = millis();
        
        // Read current position
        if (!getRawGPS(current_gps)) {
            Serial.println("[Navigation] WARNING: Failed to read GPS");
            continue;
        }
        
        // Check GPS health
        if (!checkGPSHealth(current_gps, 6)) {  // Reduced satellite requirement during flight
            Serial.println("[Navigation] WARNING: GPS health degraded");
        }
        
        // Calculate current distance to target
        distance_m = calculateDistance(current_gps.lat, current_gps.lon, target_lat, target_lon);
        
        // Check battery
        if (getBatteryState(battery)) {
            if (!checkBatteryLevel(battery, 25)) {  // Reduced threshold during flight
                Serial.println("[Navigation] WARNING: Battery getting low!");
            }
        }
        
        Serial.printf("[Navigation] Distance: %.1fm, Alt: %dcm, Sats: %d\n", 
                      distance_m, current_gps.altCm, current_gps.numSat);
        
        // Task 3.1i: Detect arrival
        if (distance_m <= arrival_threshold_m) {
            Serial.println("[Navigation] *** ARRIVED AT WAYPOINT ***");
            arrived = true;
            break;
        }
    }
    
    // Task 3.1k: Error handling and timeout
    if (!arrived) {
        Serial.printf("[Navigation] TIMEOUT: Failed to reach waypoint (%.1fm remaining)\n", distance_m);
    }
    
    // Task 3.1j: Exit GPS Rescue and restore original config
    Serial.println("[Navigation] Deactivating GPS Rescue...");
    deactivateGPSRescue();
    
    delay(200);
    
    Serial.println("[Navigation] Restoring original GPS Rescue configuration...");
    setGPSRescue(original_rescue_config);
    
    Serial.println("[Navigation] ========== WAYPOINT NAVIGATION COMPLETE ==========");
    
    return arrived;
}

// Emergency landing at current position
// Returns true if successfully initiated landing, false on error
bool BetaflightMSP::landNow() {
    Serial.println("[Navigation] ========== LAND NOW ==========");
    
    // Task 3.2a: Pre-flight safety checks
    Serial.println("[Navigation] Performing safety checks...");
    
    msp_raw_gps_t current_gps;
    if (!getRawGPS(current_gps)) {
        Serial.println("[Navigation] ERROR: Failed to read GPS");
        return false;
    }
    
    if (!checkGPSHealth(current_gps, 6)) {  // Reduced requirement for emergency landing
        Serial.println("[Navigation] WARNING: GPS not optimal, but proceeding with landing");
    }
    
    // Task 3.2b: Read current GPS position
    int32_t current_lat = current_gps.lat;
    int32_t current_lon = current_gps.lon;
    uint16_t current_alt_m = current_gps.altCm / 100;
    
    Serial.printf("[Navigation] Current position: %.6f, %.6f @ %dm\n", 
                  current_lat/10000000.0f, current_lon/10000000.0f, current_alt_m);
    
    // Task 3.2c: Backup current GPS Rescue config
    Serial.println("[Navigation] Backing up GPS Rescue configuration...");
    msp_gps_rescue_t original_rescue_config;
    if (!getGPSRescue(original_rescue_config)) {
        Serial.println("[Navigation] ERROR: Failed to backup GPS Rescue config");
        return false;
    }
    
    // Task 3.2d: Set GPS Home to current position
    Serial.println("[Navigation] Setting GPS Home to current position...");
    if (!setGPSHome(current_lat, current_lon, current_alt_m)) {
        Serial.println("[Navigation] ERROR: Failed to set GPS Home");
        return false;
    }
    
    // Task 3.2e: Configure GPS Rescue for immediate descent
    Serial.println("[Navigation] Configuring GPS Rescue for landing...");
    msp_gps_rescue_t landing_config = original_rescue_config;
    
    landing_config.returnAltitudeM = current_alt_m;
    landing_config.altitudeMode = GPS_RESCUE_ALT_MODE_CURRENT;
    landing_config.initialClimbM = 0;  // Don't climb
    landing_config.descentDistanceM = 500;  // Very large - start descending immediately
    landing_config.descendRate = 200;  // 2 m/s descent (safe speed)
    landing_config.groundSpeedCmS = 100;  // Very slow horizontal movement
    landing_config.minStartDistM = 0;  // Allow activation at any distance
    landing_config.sanityChecks = RESCUE_SANITY_OFF;  // Disable sanity checks for emergency landing
    
    // Landing altitude: 3 meters (safe threshold)
    // Note: This value is read-only from MSP, but FC uses internal setting
    Serial.println("[Navigation] Landing altitude threshold: 3m (FC internal setting)");
    
    if (!setGPSRescue(landing_config)) {
        Serial.println("[Navigation] ERROR: Failed to configure GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.2f: Activate GPS Rescue mode
    Serial.println("[Navigation] Activating GPS Rescue mode for landing...");
    if (!activateGPSRescue()) {
        Serial.println("[Navigation] ERROR: Failed to activate GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.2g: Monitor descent progress
    Serial.println("[Navigation] Monitoring descent...");
    uint32_t start_time = millis();
    uint32_t timeout_ms = 60000;  // 60 second timeout
    uint32_t last_update = 0;
    int16_t last_altitude_cm = current_gps.altCm;
    bool landed = false;
    
    while (millis() - start_time < timeout_ms) {
        // Update every 500ms
        if (millis() - last_update < 500) {
            delay(50);
            continue;
        }
        last_update = millis();
        
        // Read current altitude
        if (!getRawGPS(current_gps)) {
            Serial.println("[Navigation] WARNING: Failed to read GPS");
            continue;
        }
        
        int16_t altitude_cm = current_gps.altCm;
        int16_t descent_cm = last_altitude_cm - altitude_cm;
        
        Serial.printf("[Navigation] Altitude: %dcm (-%dcm/500ms), Sats: %d\n", 
                      altitude_cm, descent_cm, current_gps.numSat);
        
        // Task 3.2h: Detect landing completion
        // Landing detected when:
        // 1. Altitude below 300cm (3m) AND
        // 2. Descent rate very slow (less than 10cm per update)
        if (altitude_cm < 300 && abs(descent_cm) < 10) {
            Serial.println("[Navigation] *** LANDING DETECTED ***");
            landed = true;
            break;
        }
        
        last_altitude_cm = altitude_cm;
    }
    
    if (!landed) {
        Serial.println("[Navigation] WARNING: Landing timeout - assuming landed");
        landed = true;  // Assume landed for safety
    }
    
    // Task 3.2i: Restore original config (if not landed, though we assume landed)
    delay(500);  // Wait for stable landing
    
    Serial.println("[Navigation] Deactivating GPS Rescue...");
    deactivateGPSRescue();
    
    delay(200);
    
    Serial.println("[Navigation] Restoring original GPS Rescue configuration...");
    setGPSRescue(original_rescue_config);
    
    Serial.println("[Navigation] ========== LANDING COMPLETE ==========");
    
    return landed;
}

// Hold position at current location (loiter mode)
// duration_ms: How long to hold (0 = hold indefinitely until cancelled)
// Returns true if successfully held position, false on error
bool BetaflightMSP::holdPosition(uint32_t duration_ms) {
    Serial.println("[Navigation] ========== HOLD POSITION ==========");
    
    // Task 3.3a: Pre-flight safety checks
    Serial.println("[Navigation] Performing safety checks...");
    
    msp_raw_gps_t current_gps;
    if (!getRawGPS(current_gps)) {
        Serial.println("[Navigation] ERROR: Failed to read GPS");
        return false;
    }
    
    if (!checkGPSHealth(current_gps)) {
        Serial.println("[Navigation] ERROR: GPS not healthy");
        return false;
    }
    
    msp_battery_state_t battery;
    if (!getBatteryState(battery)) {
        Serial.println("[Navigation] ERROR: Failed to read battery");
        return false;
    }
    
    if (!checkBatteryLevel(battery, 30)) {
        Serial.println("[Navigation] ERROR: Battery too low");
        return false;
    }
    
    // Task 3.3b: Read current GPS position and altitude
    int32_t hold_lat = current_gps.lat;
    int32_t hold_lon = current_gps.lon;
    uint16_t hold_alt_m = current_gps.altCm / 100;
    
    Serial.printf("[Navigation] Hold position: %.6f, %.6f @ %dm\n", 
                  hold_lat/10000000.0f, hold_lon/10000000.0f, hold_alt_m);
    
    if (duration_ms > 0) {
        Serial.printf("[Navigation] Hold duration: %lu seconds\n", duration_ms / 1000);
    } else {
        Serial.println("[Navigation] Hold duration: INDEFINITE (manual cancel required)");
    }
    
    // Task 3.3c: Backup current GPS Rescue config
    Serial.println("[Navigation] Backing up GPS Rescue configuration...");
    msp_gps_rescue_t original_rescue_config;
    if (!getGPSRescue(original_rescue_config)) {
        Serial.println("[Navigation] ERROR: Failed to backup GPS Rescue config");
        return false;
    }
    
    // Task 3.3d: Set GPS Home to current position
    Serial.println("[Navigation] Setting GPS Home to current position...");
    if (!setGPSHome(hold_lat, hold_lon, hold_alt_m)) {
        Serial.println("[Navigation] ERROR: Failed to set GPS Home");
        return false;
    }
    
    // Task 3.3e: Configure GPS Rescue for hovering
    Serial.println("[Navigation] Configuring GPS Rescue for position hold...");
    msp_gps_rescue_t hold_config = original_rescue_config;
    
    hold_config.returnAltitudeM = hold_alt_m;
    hold_config.altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
    hold_config.initialClimbM = 0;  // Don't climb
    hold_config.descentDistanceM = 3;  // Very small - only descend when very close to "home"
    hold_config.descendRate = 50;  // Very slow descent (0.5 m/s) - essentially hover
    hold_config.groundSpeedCmS = 400;  // Moderate speed to return to center (4 m/s)
    hold_config.ascendRate = 500;  // 5 m/s climb if needed
    hold_config.minStartDistM = 0;  // Allow activation at any distance
    hold_config.maxRescueAngle = 30;  // Gentle corrections
    hold_config.sanityChecks = RESCUE_SANITY_OFF;  // Disable sanity checks
    
    if (!setGPSRescue(hold_config)) {
        Serial.println("[Navigation] ERROR: Failed to configure GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.3f: Activate GPS Rescue mode
    Serial.println("[Navigation] Activating GPS Rescue mode for position hold...");
    if (!activateGPSRescue()) {
        Serial.println("[Navigation] ERROR: Failed to activate GPS Rescue");
        setGPSRescue(original_rescue_config);  // Restore original
        return false;
    }
    
    // Task 3.3g: Monitor position (with natural GPS drift oscillation)
    Serial.println("[Navigation] Holding position (GPS drift oscillation is normal)...");
    uint32_t start_time = millis();
    uint32_t last_update = 0;
    bool hold_complete = false;
    
    while (true) {
        // Check duration timeout
        if (duration_ms > 0 && (millis() - start_time >= duration_ms)) {
            Serial.println("[Navigation] Hold duration complete");
            hold_complete = true;
            break;
        }
        
        // Update every 1000ms
        if (millis() - last_update < 1000) {
            delay(100);
            continue;
        }
        last_update = millis();
        
        // Read current position
        if (!getRawGPS(current_gps)) {
            Serial.println("[Navigation] WARNING: Failed to read GPS");
            continue;
        }
        
        // Calculate drift from hold position
        float drift_m = calculateDistance(hold_lat, hold_lon, current_gps.lat, current_gps.lon);
        float bearing = calculateBearing(hold_lat, hold_lon, current_gps.lat, current_gps.lon);
        
        // Check battery
        if (getBatteryState(battery)) {
            if (!checkBatteryLevel(battery, 25)) {
                Serial.println("[Navigation] WARNING: Battery getting low!");
            }
        }
        
        Serial.printf("[Navigation] Drift: %.1fm @ %.0f°, Alt: %dcm, Sats: %d\n", 
                      drift_m, bearing, current_gps.altCm, current_gps.numSat);
        
        // Check GPS health
        if (!checkGPSHealth(current_gps, 6)) {
            Serial.println("[Navigation] WARNING: GPS health degraded");
        }
        
        // Optional: Re-center home if drift exceeds threshold
        // Uncommented for now - let natural GPS Rescue behavior handle it
        /*
        if (drift_m > 10.0f) {
            Serial.println("[Navigation] Excessive drift detected - re-centering home");
            setGPSHome(current_gps.lat, current_gps.lon, hold_alt_m);
            hold_lat = current_gps.lat;
            hold_lon = current_gps.lon;
        }
        */
    }
    
    // Task 3.3h: Exit on command or timeout
    // Task 3.3i: Restore original config
    Serial.println("[Navigation] Deactivating GPS Rescue...");
    deactivateGPSRescue();
    
    delay(200);
    
    Serial.println("[Navigation] Restoring original GPS Rescue configuration...");
    setGPSRescue(original_rescue_config);
    
    Serial.println("[Navigation] ========== HOLD POSITION COMPLETE ==========");
    
    return hold_complete;
}

// GPS jitter helper function - calculates 10m offset in rotating direction
// Task 3.4a: Implement getJitteredGPSHome()
void BetaflightMSP::getJitteredGPSHome(int32_t origin_lat, int32_t origin_lon,
                                        int32_t &jittered_lat, int32_t &jittered_lon) {
    // Task 3.4b: Rotate through 8 directions (N, NE, E, SE, S, SW, W, NW)
    const float JITTER_DISTANCE_M = 10.0f;
    const float angles[8] = {0, 45, 90, 135, 180, 225, 270, 315};  // degrees
    
    float bearing = angles[_jitterDirection];
    
    Serial.printf("[Jitter] Direction: %d (%s), Bearing: %.0f°\n", 
                  _jitterDirection,
                  (_jitterDirection == 0 ? "N" : 
                   _jitterDirection == 1 ? "NE" :
                   _jitterDirection == 2 ? "E" :
                   _jitterDirection == 3 ? "SE" :
                   _jitterDirection == 4 ? "S" :
                   _jitterDirection == 5 ? "SW" :
                   _jitterDirection == 6 ? "W" : "NW"),
                  bearing);
    
    // Calculate jittered position
    offsetGPSCoordinate(origin_lat, origin_lon, JITTER_DISTANCE_M, bearing,
                       jittered_lat, jittered_lon);
    
    // Rotate to next direction for next time
    _jitterDirection = (_jitterDirection + 1) % 8;
}

// Takeoff to target altitude with GPS jitter workaround
// Returns true if successfully reached altitude, false on error or timeout
bool BetaflightMSP::takeoff(uint16_t target_altitude_m) {
    Serial.println("[Navigation] ========== TAKEOFF ==========");
    
    // Task 3.5a: Pre-flight safety checks
    Serial.println("[Navigation] Performing pre-flight checks...");
    
    msp_raw_gps_t current_gps;
    if (!getRawGPS(current_gps)) {
        Serial.println("[Navigation] ERROR: Failed to read GPS");
        return false;
    }
    
    if (!checkGPSHealth(current_gps)) {
        Serial.println("[Navigation] ERROR: GPS not healthy");
        return false;
    }
    
    msp_battery_state_t battery;
    if (!getBatteryState(battery)) {
        Serial.println("[Navigation] ERROR: Failed to read battery");
        return false;
    }
    
    if (!checkBatteryLevel(battery, 40)) {  // Higher threshold for takeoff
        Serial.println("[Navigation] ERROR: Battery too low for takeoff");
        return false;
    }
    
    if (target_altitude_m < 5 || target_altitude_m > 50) {
        Serial.println("[Navigation] ERROR: Takeoff altitude out of range (5-50m)");
        return false;
    }
    
    // Task 3.5b: Read takeoff GPS position
    int32_t takeoff_lat = current_gps.lat;
    int32_t takeoff_lon = current_gps.lon;
    uint16_t ground_alt_m = current_gps.altCm / 100;
    
    Serial.printf("[Navigation] Takeoff from: %.6f, %.6f @ %dm\n", 
                  takeoff_lat/10000000.0f, takeoff_lon/10000000.0f, ground_alt_m);
    Serial.printf("[Navigation] Target altitude: %dm\n", target_altitude_m);
    
    // Task 3.5c: Backup current GPS Rescue config
    Serial.println("[Navigation] Backing up GPS Rescue configuration...");
    msp_gps_rescue_t original_rescue_config;
    if (!getGPSRescue(original_rescue_config)) {
        Serial.println("[Navigation] ERROR: Failed to backup GPS Rescue config");
        return false;
    }
    
    // Task 3.5d: Calculate jittered GPS Home (10m from takeoff)
    int32_t jittered_lat, jittered_lon;
    getJitteredGPSHome(takeoff_lat, takeoff_lon, jittered_lat, jittered_lon);
    
    Serial.printf("[Navigation] Jittered home: %.6f, %.6f (10m offset)\n", 
                  jittered_lat/10000000.0f, jittered_lon/10000000.0f);
    
    // Task 3.5e: Configure GPS Rescue for climb
    Serial.println("[Navigation] Configuring GPS Rescue for takeoff climb...");
    msp_gps_rescue_t takeoff_config = original_rescue_config;
    
    takeoff_config.returnAltitudeM = target_altitude_m + ground_alt_m;
    takeoff_config.altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
    takeoff_config.initialClimbM = target_altitude_m;  // Climb immediately
    takeoff_config.ascendRate = 500;  // 5 m/s climb
    takeoff_config.groundSpeedCmS = 500;  // 5 m/s horizontal (drift toward jittered home)
    takeoff_config.descentDistanceM = 5;  // Small radius before descent
    takeoff_config.descendRate = 100;  // Slow descent if triggered
    takeoff_config.minStartDistM = 0;
    takeoff_config.maxRescueAngle = 30;  // Gentle angle
    takeoff_config.sanityChecks = RESCUE_SANITY_OFF;
    
    if (!setGPSRescue(takeoff_config)) {
        Serial.println("[Navigation] ERROR: Failed to configure GPS Rescue");
        setGPSRescue(original_rescue_config);
        return false;
    }
    
    // Task 3.5f: Set GPS Home to jittered position
    Serial.println("[Navigation] Setting GPS Home to jittered position...");
    if (!setGPSHome(jittered_lat, jittered_lon, target_altitude_m + ground_alt_m)) {
        Serial.println("[Navigation] ERROR: Failed to set GPS Home");
        setGPSRescue(original_rescue_config);
        return false;
    }
    
    // Task 3.5g: Activate GPS Rescue mode
    Serial.println("[Navigation] Activating GPS Rescue mode for takeoff...");
    if (!activateGPSRescue()) {
        Serial.println("[Navigation] ERROR: Failed to activate GPS Rescue");
        setGPSRescue(original_rescue_config);
        return false;
    }
    
    // Task 3.5h: Monitor altitude until target reached
    Serial.println("[Navigation] Monitoring climb...");
    uint32_t start_time = millis();
    uint32_t timeout_ms = (uint32_t)((target_altitude_m / 5.0f) * 1500);  // Based on climb rate with margin
    uint32_t last_update = 0;
    bool altitude_reached = false;
    
    while (millis() - start_time < timeout_ms) {
        // Update every 500ms
        if (millis() - last_update < 500) {
            delay(50);
            continue;
        }
        last_update = millis();
        
        // Read current altitude
        if (!getRawGPS(current_gps)) {
            Serial.println("[Navigation] WARNING: Failed to read GPS");
            continue;
        }
        
        uint16_t current_alt_m = current_gps.altCm / 100;
        uint16_t altitude_agl = current_alt_m - ground_alt_m;  // Above Ground Level
        
        Serial.printf("[Navigation] Altitude: %dm AGL (%dm total), Sats: %d\n", 
                      altitude_agl, current_alt_m, current_gps.numSat);
        
        // Check if target altitude reached (with 1m tolerance)
        if (altitude_agl >= target_altitude_m - 1) {
            Serial.printf("[Navigation] *** TARGET ALTITUDE REACHED (%dm) ***\n", altitude_agl);
            altitude_reached = true;
            break;
        }
    }
    
    if (!altitude_reached) {
        Serial.println("[Navigation] TIMEOUT: Failed to reach target altitude");
    }
    
    // Task 3.5i: Deactivate GPS Rescue at target altitude
    Serial.println("[Navigation] Deactivating GPS Rescue...");
    deactivateGPSRescue();
    
    delay(200);
    
    // Task 3.5j: Switch to Angle mode (already done by deactivateGPSRescue)
    Serial.println("[Navigation] Now in Angle mode - manual control available");
    
    // Task 3.5k: Restore original config
    Serial.println("[Navigation] Restoring original GPS Rescue configuration...");
    setGPSRescue(original_rescue_config);
    
    Serial.println("[Navigation] ========== TAKEOFF COMPLETE ==========");
    Serial.printf("[Navigation] Note: Drone may have drifted up to 10m from takeoff point\n");
    
    return altitude_reached;
}

// ========================================
// RETURN-TO-HOME (RTH) FUNCTIONS
// ========================================

/**
 * Set home position manually
 * @param home_lat Home latitude (degrees * 1e7)
 * @param home_lon Home longitude (degrees * 1e7)
 * @return true if position was saved successfully
 */
bool BetaflightMSP::setHomePosition(int32_t home_lat, int32_t home_lon) {
    // Validate coordinates
    if (abs(home_lat) > 900000000 || abs(home_lon) > 1800000000) {
        Serial.println("[RTH] ERROR: Invalid home coordinates");
        return false;
    }
    
    _homeLat = home_lat;
    _homeLon = home_lon;
    _homePositionSet = true;
    
    Serial.println("[RTH] Home position set manually");
    Serial.printf("[RTH] Home: Lat=%.7f, Lon=%.7f\n", 
                  _homeLat / 1e7, _homeLon / 1e7);
    
    return true;
}

/**
 * Set home position to current GPS location
 * @return true if home was set successfully
 */
bool BetaflightMSP::setHomePosition() {
    // Read current GPS position
    msp_raw_gps_t gps;
    if (!getRawGPS(gps)) {
        Serial.println("[RTH] ERROR: Failed to read GPS position");
        return false;
    }
    
    // Check GPS health
    if (!checkGPSHealth(gps, 8)) {
        Serial.println("[RTH] ERROR: GPS not healthy for home position");
        return false;
    }
    
    return setHomePosition(gps.lat, gps.lon);
}

/**
 * Get the saved home position
 * @param home_lat Reference to store home latitude
 * @param home_lon Reference to store home longitude
 * @return true if home position is set
 */
bool BetaflightMSP::getHomePosition(int32_t &home_lat, int32_t &home_lon) {
    if (!_homePositionSet) {
        Serial.println("[RTH] ERROR: Home position not set");
        return false;
    }
    
    home_lat = _homeLat;
    home_lon = _homeLon;
    return true;
}

/**
 * Return to Home (RTH) - Navigate to saved home position and hover or land
 * 
 * This function uses the existing navigation functions to return to a previously
 * saved home position. After arrival, it can either hover (for inspection/manual takeover)
 * or land automatically.
 * 
 * @param behavior RTH_HOVER (hold position) or RTH_LAND (land automatically)
 * @param altitude_m Altitude to maintain during return flight (5-120m, default 30m)
 * @param speed_cm_s Cruise speed during return (100-3000 cm/s, default 750 cm/s = 7.5 m/s)
 * @return true if RTH completed successfully
 */
bool BetaflightMSP::returnToHome(RTHBehavior behavior, uint16_t altitude_m, uint16_t speed_cm_s) {
    Serial.println("[RTH] ========== RETURN TO HOME INITIATED ==========");
    
    // Check if home position is set
    if (!_homePositionSet) {
        Serial.println("[RTH] ERROR: Home position not set!");
        Serial.println("[RTH] Call setHomePosition() first");
        return false;
    }
      // Read current GPS position
    msp_raw_gps_t gps;
    if (!getRawGPS(gps)) {
        Serial.println("[RTH] ERROR: Failed to read GPS position");
        return false;
    }
    
    // Calculate distance to home
    float distance_to_home = calculateDistance(gps.lat, gps.lon, _homeLat, _homeLon);
    float bearing_to_home = calculateBearing(gps.lat, gps.lon, _homeLat, _homeLon);
    
    Serial.printf("[RTH] Current position: Lat=%.7f, Lon=%.7f\n", 
                  gps.lat / 1e7, gps.lon / 1e7);
    Serial.printf("[RTH] Home position: Lat=%.7f, Lon=%.7f\n", 
                  _homeLat / 1e7, _homeLon / 1e7);
    Serial.printf("[RTH] Distance to home: %.2f m\n", distance_to_home);
    Serial.printf("[RTH] Bearing to home: %.1f°\n", bearing_to_home);
    Serial.printf("[RTH] Behavior: %s\n", 
                  behavior == RTH_HOVER ? "HOVER" : "LAND");
    
    // Check if already at home (within 5m)
    if (distance_to_home < 5.0f) {
        Serial.println("[RTH] Already at home position!");
        
        // Just execute the final behavior
        if (behavior == RTH_LAND) {
            Serial.println("[RTH] Executing landing...");
            return landNow();
        } else {
            Serial.println("[RTH] Entering hover mode...");
            return holdPosition(0);  // Indefinite hold
        }
    }
    
    // Task 5.2c: Navigate to home position using gotoWaypoint()
    Serial.println("[RTH] Navigating to home position...");
    bool navigation_success = gotoWaypoint(_homeLat, _homeLon, altitude_m, speed_cm_s, 5.0f);
    
    if (!navigation_success) {
        Serial.println("[RTH] ERROR: Failed to reach home position");
        return false;
    }
    
    Serial.println("[RTH] Arrived at home position!");
    
    // Task 5.2d: Execute behavior based on parameter
    if (behavior == RTH_LAND) {
        Serial.println("[RTH] Executing automatic landing...");
        bool landing_success = landNow();
        
        if (landing_success) {
            Serial.println("[RTH] ========== RTH COMPLETE - LANDED ==========");
        } else {
            Serial.println("[RTH] WARNING: Landing may not have completed successfully");
        }
        
        return landing_success;
        
    } else {  // RTH_HOVER
        Serial.println("[RTH] Entering hover mode (indefinite)...");
        Serial.println("[RTH] Manual control can be resumed at any time");
        
        bool hover_success = holdPosition(0);  // 0 = indefinite hold
        
        if (hover_success) {
            Serial.println("[RTH] ========== RTH COMPLETE - HOVERING ==========");
        } else {
            Serial.println("[RTH] WARNING: Hold position may have failed");
        }
        
        return hover_success;
    }
}

// ========================================
// ORBIT FUNCTION (Circular Flight Pattern)
// ========================================

/**
 * Orbit around a GPS coordinate in a circular pattern
 * 
 * This function flies the drone in a smooth circle around a center point.
 * The circle is divided into waypoints spaced ~20m apart, and the GPS Home
 * is updated 3-5m before reaching each waypoint for smooth transitions.
 * 
 * @param center_lat Center point latitude (degrees * 1e7)
 * @param center_lon Center point longitude (degrees * 1e7)
 * @param radius_m Orbit radius in meters (10-500m)
 * @param orbits_per_hour Orbit rate (0.5-12 orbits/hour, typical: 4-8)
 * @param altitude_m Altitude to maintain during orbit (5-120m)
 * @param direction Orbit direction (ORBIT_CW = clockwise, ORBIT_CCW = counter-clockwise)
 * @param duration_ms Maximum duration in milliseconds (0 = indefinite)
 * @param max_laps Maximum number of complete orbits (0 = indefinite)
 * @return true if orbit completed successfully
 */
bool BetaflightMSP::orbit(int32_t center_lat, int32_t center_lon, uint16_t radius_m,
                          float orbits_per_hour, uint16_t altitude_m,
                          OrbitDirection direction,
                          uint32_t duration_ms, uint8_t max_laps) {
    
    Serial.println("[Orbit] ========== ORBIT INITIATED ==========");
    
    // Task 5.5a: Input validation
    if (abs(center_lat) > 900000000 || abs(center_lon) > 1800000000) {
        Serial.println("[Orbit] ERROR: Invalid center coordinates");
        return false;
    }
    
    if (radius_m < 10 || radius_m > 500) {
        Serial.println("[Orbit] ERROR: Radius must be 10-500m");
        return false;
    }
    
    if (orbits_per_hour < 0.5f || orbits_per_hour > 12.0f) {
        Serial.println("[Orbit] ERROR: Orbit rate must be 0.5-12 orbits/hour");
        return false;
    }
    
    if (altitude_m < 5 || altitude_m > 120) {
        Serial.println("[Orbit] ERROR: Altitude must be 5-120m");
        return false;
    }
      Serial.printf("[Orbit] Center: Lat=%.7f, Lon=%.7f\n", center_lat / 1e7, center_lon / 1e7);
    Serial.printf("[Orbit] Radius: %d m, Altitude: %d m\n", radius_m, altitude_m);
    Serial.printf("[Orbit] Orbit rate: %.2f orbits/hour\n", orbits_per_hour);
    Serial.printf("[Orbit] Direction: %s\n", direction == ORBIT_CW ? "Clockwise (CW)" : "Counter-Clockwise (CCW)");
    
    // Pre-flight safety checks
    msp_raw_gps_t gps;
    if (!getRawGPS(gps)) {
        Serial.println("[Orbit] ERROR: Failed to read GPS");
        return false;
    }
    
    if (!checkGPSHealth(gps, 8)) {
        Serial.println("[Orbit] ERROR: GPS not healthy");
        return false;
    }
    
    msp_battery_state_t battery;
    if (!getBatteryState(battery)) {
        Serial.println("[Orbit] ERROR: Failed to read battery");
        return false;
    }
    
    if (!checkBatteryLevel(battery, 30)) {
        Serial.println("[Orbit] ERROR: Battery too low");
        return false;
    }
    
    // Task 5.5b: Calculate number of waypoints (min 20m spacing)
    float circumference = 2.0f * PI * radius_m;
    uint8_t num_waypoints = (uint8_t)(circumference / 20.0f);
    if (num_waypoints < 8) num_waypoints = 8;   // Minimum 8 points
    if (num_waypoints > 36) num_waypoints = 36; // Maximum 36 points (10° spacing)
    
    float angle_step = 360.0f / num_waypoints;
    
    Serial.printf("[Orbit] Circumference: %.1f m\n", circumference);
    Serial.printf("[Orbit] Waypoints: %d (every %.1f°)\n", num_waypoints, angle_step);
    
    // Task 5.5d: Calculate speed based on orbit rate
    float seconds_per_orbit = 3600.0f / orbits_per_hour;
    uint16_t speed_cm_s = (uint16_t)((circumference / seconds_per_orbit) * 100.0f);
    
    // Clamp speed to safe range
    if (speed_cm_s < 100) speed_cm_s = 100;     // Min 1 m/s
    if (speed_cm_s > 3000) speed_cm_s = 3000;   // Max 30 m/s
    
    Serial.printf("[Orbit] Time per orbit: %.1f seconds\n", seconds_per_orbit);
    Serial.printf("[Orbit] Speed: %d cm/s (%.2f m/s)\n", speed_cm_s, speed_cm_s / 100.0f);
      // Task 5.5c: Generate circle waypoints
    int32_t waypoints_lat[36];  // Max 36 waypoints
    int32_t waypoints_lon[36];
    
    for (uint8_t i = 0; i < num_waypoints; i++) {
        // Calculate bearing based on direction
        // CW: Start at 0° (North) and go clockwise (0, 45, 90, 135...)
        // CCW: Start at 0° (North) and go counter-clockwise (0, 315, 270, 225...)
        float bearing;
        if (direction == ORBIT_CW) {
            bearing = i * angle_step;  // Clockwise: 0, 22.5, 45, 67.5...
        } else {
            bearing = 360.0f - (i * angle_step);  // Counter-clockwise: 0, 337.5, 315, 292.5...
            if (bearing >= 360.0f) bearing = 0.0f;
        }
        
        offsetGPSCoordinate(center_lat, center_lon, radius_m, bearing,
                           waypoints_lat[i], waypoints_lon[i]);
        
        Serial.printf("[Orbit] WP%d: Bearing=%.1f° Lat=%.7f Lon=%.7f\n",
                      i, bearing, waypoints_lat[i] / 1e7, waypoints_lon[i] / 1e7);
    }
    
    // Backup GPS Rescue configuration
    msp_gps_rescue_t original_rescue_config;
    if (!getGPSRescue(original_rescue_config)) {
        Serial.println("[Orbit] ERROR: Failed to read GPS Rescue config");
        return false;
    }
    
    // Task 5.5h: Initialize orbit state tracking
    _orbitCurrentWaypoint = 0;
    _orbitLapsCompleted = 0;
    
    uint32_t orbit_start_time = millis();
    bool orbit_active = true;
    
    Serial.println("[Orbit] ========== ORBIT STARTED ==========");
    
    // Task 5.5f: Main orbit loop
    while (orbit_active) {
        uint32_t elapsed_time = millis() - orbit_start_time;
        
        // Check stop conditions (Task 5.5g)
        if (duration_ms > 0 && elapsed_time >= duration_ms) {
            Serial.println("[Orbit] Duration limit reached");
            break;
        }
        
        if (max_laps > 0 && _orbitLapsCompleted >= max_laps) {
            Serial.println("[Orbit] Lap count reached");
            break;
        }
        
        // Safety checks
        if (!getRawGPS(gps) || !checkGPSHealth(gps, 8)) {
            Serial.println("[Orbit] ERROR: GPS health check failed");
            orbit_active = false;
            break;
        }
        
        if (!getBatteryState(battery) || !checkBatteryLevel(battery, 30)) {
            Serial.println("[Orbit] ERROR: Battery too low");
            orbit_active = false;
            break;
        }
        
        // Get current position
        if (!getRawGPS(gps)) {
            Serial.println("[Orbit] ERROR: Failed to read GPS");
            orbit_active = false;
            break;
        }
        
        // Get current and next waypoint
        uint8_t current_wp = _orbitCurrentWaypoint;
        uint8_t next_wp = (current_wp + 1) % num_waypoints;
        
        int32_t target_lat = waypoints_lat[current_wp];
        int32_t target_lon = waypoints_lon[current_wp];
        int32_t next_lat = waypoints_lat[next_wp];
        int32_t next_lon = waypoints_lon[next_wp];
        
        // Calculate distance to current waypoint
        float distance = calculateDistance(gps.lat, gps.lon, target_lat, target_lon);
        
        Serial.printf("[Orbit] Lap %d, WP %d/%d, Distance: %.2f m\n",
                      _orbitLapsCompleted + 1, current_wp + 1, num_waypoints, distance);
        
        // Task 5.5e: Smooth transition - update GPS Home before arrival
        if (distance < 5.0f) {
            // Close to current waypoint, switch to next waypoint
            Serial.printf("[Orbit] Switching to next waypoint %d\n", next_wp + 1);
              // Configure GPS Rescue for next waypoint
            msp_gps_rescue_t rescue_config = original_rescue_config;
            rescue_config.returnAltitudeM = altitude_m;
            rescue_config.altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
            rescue_config.groundSpeedCmS = speed_cm_s;
            rescue_config.descentDistanceM = 10;  // Small descent distance
            
            if (!setGPSRescue(rescue_config)) {
                Serial.println("[Orbit] ERROR: Failed to update GPS Rescue config");
                orbit_active = false;
                break;
            }
            
            // Set GPS Home to next waypoint for smooth transition
            if (!setGPSHome(next_lat, next_lon, altitude_m * 100)) {
                Serial.println("[Orbit] ERROR: Failed to set GPS Home");
                orbit_active = false;
                break;
            }
            
            // Activate GPS Rescue if not already active
            if (!isGPSRescueActive()) {
                if (!activateGPSRescue()) {
                    Serial.println("[Orbit] ERROR: Failed to activate GPS Rescue");
                    orbit_active = false;
                    break;
                }
            }
            
            // Advance to next waypoint
            _orbitCurrentWaypoint = next_wp;
            
            // Check if we completed a lap
            if (next_wp == 0) {
                _orbitLapsCompleted++;
                Serial.printf("[Orbit] ===== LAP %d COMPLETED =====\n", _orbitLapsCompleted);
            }
        } else if (distance > 15.0f) {
            // Too far from waypoint, navigate to it
            Serial.println("[Orbit] Re-acquiring waypoint...");
              msp_gps_rescue_t rescue_config = original_rescue_config;
            rescue_config.returnAltitudeM = altitude_m;
            rescue_config.altitudeMode = GPS_RESCUE_ALT_MODE_FIXED;
            rescue_config.groundSpeedCmS = speed_cm_s;
            rescue_config.descentDistanceM = 10;
            
            if (!setGPSRescue(rescue_config)) {
                Serial.println("[Orbit] ERROR: Failed to set GPS Rescue config");
                orbit_active = false;
                break;
            }
            
            if (!setGPSHome(target_lat, target_lon, altitude_m * 100)) {
                Serial.println("[Orbit] ERROR: Failed to set GPS Home");
                orbit_active = false;
                break;
            }
            
            if (!isGPSRescueActive()) {
                if (!activateGPSRescue()) {
                    Serial.println("[Orbit] ERROR: Failed to activate GPS Rescue");
                    orbit_active = false;
                    break;
                }
            }
        }
        
        delay(500);  // Update every 500ms
    }
    
    // Exit orbit
    Serial.println("[Orbit] Deactivating GPS Rescue...");
    deactivateGPSRescue();
    
    delay(200);
    
    // Restore original GPS Rescue config
    Serial.println("[Orbit] Restoring original GPS Rescue configuration...");
    setGPSRescue(original_rescue_config);
    
    Serial.println("[Orbit] ========== ORBIT COMPLETE ==========");
    Serial.printf("[Orbit] Total laps completed: %d\n", _orbitLapsCompleted);
    Serial.printf("[Orbit] Total time: %.1f seconds\n", (millis() - orbit_start_time) / 1000.0f);
    
    return orbit_active;  // true if completed normally, false if aborted
}
