/// DBC file: TopGun_12_38am_11_16_2015.dbc    Self node: MOTORIO
/// This file should be included by a source file, for example: #include "generated.c"
#include <stdbool.h>
#include <stdint.h>



/// Missing in Action structure
typedef struct {
    uint32_t is_mia : 1;          ///< Missing in action flag
    uint32_t mia_counter_ms : 31; ///< Missing in action counter
} mia_info_t;

/// CAN message header structure
typedef struct { 
    uint32_t mid; ///< Message ID of the message
    uint8_t  dlc; ///< Data length of the message
} msg_hdr_t; 

static const msg_hdr_t DRIVER_TX_DRIVER_KILL_SWITCH_HDR =     {    0, 0 };
static const msg_hdr_t DRIVER_TX_DRIVER_RESET_HDR =           {    1, 0 };
static const msg_hdr_t DRIVER_TX_DRIVER_SYNC_ACK_HDR =        {    2, 0 };
static const msg_hdr_t MOTORIO_TX_MOTORIO_SYNC_HDR =          {    3, 0 };
static const msg_hdr_t SENSOR_TX_SENSOR_SYNC_HDR =            {    4, 0 };
static const msg_hdr_t BLUETOOTH_TX_BLUETOOTH_SYNC_HDR =      {    5, 0 };
static const msg_hdr_t GEO_TX_GEO_SYNC_HDR =                  {    6, 0 };
static const msg_hdr_t MOTORIO_TX_MOTORIO_HEARTBEAT_HDR =     {    7, 0 };
static const msg_hdr_t SENSOR_TX_SENSOR_HEARTBEAT_HDR =       {    8, 0 };
static const msg_hdr_t BLUETOOTH_TX_BLUETOOTH_HEARTBEAT_HDR = {    9, 0 };
static const msg_hdr_t GEO_TX_GEO_HEARTBEAT_HDR =             {   10, 0 };
static const msg_hdr_t DRIVER_TX_MOTORIO_RUNMODE_HDR =        {   11, 1 };
static const msg_hdr_t SENSOR_TX_SENSOR_SONARS_HDR =          {   12, 6 };
static const msg_hdr_t DRIVER_TX_MOTORIO_DIRECTION_HDR =      {   13, 2 };
static const msg_hdr_t DRIVER_TX_DRIVER_CHECKPOINT_REQ_HDR =  {   14, 1 };
static const msg_hdr_t BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_HDR = {   15, 1 };
static const msg_hdr_t BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_HDR = {   16, 1 };
static const msg_hdr_t DRIVER_TX_DRIVER_LOC_UPDATE_HDR =      {   17, 8 };
static const msg_hdr_t GEO_TX_GEO_SPEED_ANGLE_HDR =           {   18, 5 };
static const msg_hdr_t GEO_TX_GEO_LOC_DATA_HDR =              {   19, 8 };
static const msg_hdr_t SENSOR_TX_SENSOR_LIGHT_BAT_HDR =       {   20, 2 };


/// Message: DRIVER_KILL_SWITCH from 'DRIVER', DLC: 0 byte(s), MID: 0
typedef struct {
    uint8_t DRIVER_KILL_SWITCH_cmd;      ///< B-1:0   Destination: SENSOR,MOTORIO,BLUETOOTH,GEO

    mia_info_t mia_info;
} DRIVER_TX_DRIVER_KILL_SWITCH_t;


/// Message: DRIVER_RESET from 'DRIVER', DLC: 0 byte(s), MID: 1
typedef struct {
    uint8_t DRIVER_RESET_cmd;            ///< B-1:0   Destination: SENSOR,MOTORIO,BLUETOOTH,GEO

    mia_info_t mia_info;
} DRIVER_TX_DRIVER_RESET_t;


/// Message: DRIVER_SYNC_ACK from 'DRIVER', DLC: 0 byte(s), MID: 2
typedef struct {
    uint8_t DRIVER_SYNC_ACK_cmd;         ///< B-1:0   Destination: SENSOR,MOTORIO,BLUETOOTH,GEO

    mia_info_t mia_info;
} DRIVER_TX_DRIVER_SYNC_ACK_t;


/// Message: MOTORIO_SYNC from 'MOTORIO', DLC: 0 byte(s), MID: 3
typedef struct {
    uint8_t MOTORIO_SYNC_cmd;            ///< B-1:0   Destination: DRIVER

    mia_info_t mia_info;
} MOTORIO_TX_MOTORIO_SYNC_t;


/// Not generating 'SENSOR_TX_SENSOR_SYNC_t' since we are not the sender or a recipient of any of its signals

/// Not generating 'BLUETOOTH_TX_BLUETOOTH_SYNC_t' since we are not the sender or a recipient of any of its signals

/// Not generating 'GEO_TX_GEO_SYNC_t' since we are not the sender or a recipient of any of its signals

/// Message: MOTORIO_HEARTBEAT from 'MOTORIO', DLC: 0 byte(s), MID: 7
typedef struct {
    uint8_t MOTORIO_HEARTBEAT_cmd;       ///< B-1:0   Destination: DRIVER

    mia_info_t mia_info;
} MOTORIO_TX_MOTORIO_HEARTBEAT_t;


/// Not generating 'SENSOR_TX_SENSOR_HEARTBEAT_t' since we are not the sender or a recipient of any of its signals

/// Not generating 'BLUETOOTH_TX_BLUETOOTH_HEARTBEAT_t' since we are not the sender or a recipient of any of its signals

/// Not generating 'GEO_TX_GEO_HEARTBEAT_t' since we are not the sender or a recipient of any of its signals

/// Message: MOTORIO_RUNMODE from 'DRIVER', DLC: 1 byte(s), MID: 11
typedef struct {
    uint8_t MOTORIO_RUNMODE_cmd;         ///< B7:0  Min: 0 Max: 3   Destination: MOTORIO

    mia_info_t mia_info;
} DRIVER_TX_MOTORIO_RUNMODE_t;


/// Message: SENSOR_SONARS from 'SENSOR', DLC: 6 byte(s), MID: 12
typedef struct {
    uint8_t SENSOR_SONARS_front_left;    ///< B7:0  Min: 0 Max: 4   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_SONARS_front_right;   ///< B15:8  Min: 0 Max: 4   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_SONARS_front_center;  ///< B23:16  Min: 0 Max: 4   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_SONARS_left;          ///< B31:24  Min: 0 Max: 4   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_SONARS_right;         ///< B39:32  Min: 0 Max: 4   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_SONARS_back;          ///< B47:40  Min: 0 Max: 4   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} SENSOR_TX_SENSOR_SONARS_t;


/// Message: MOTORIO_DIRECTION from 'DRIVER', DLC: 2 byte(s), MID: 13
typedef struct {
    uint8_t MOTORIO_DIRECTION_speed_cmd; ///< B7:0  Min: 0 Max: 4   Destination: MOTORIO
    uint8_t MOTORIO_DIRECTION_turn_cmd;  ///< B15:8  Min: 0 Max: 5   Destination: MOTORIO

    mia_info_t mia_info;
} DRIVER_TX_MOTORIO_DIRECTION_t;


/// Message: DRIVER_CHECKPOINT_REQ from 'DRIVER', DLC: 1 byte(s), MID: 14
typedef struct {
    uint8_t DRIVER_CHECKPOINT_REQ_cmd;   ///< B7:0  Min: 0 Max: 3   Destination: BLUETOOTH,MOTORIO

    mia_info_t mia_info;
} DRIVER_TX_DRIVER_CHECKPOINT_REQ_t;


/// Message: BLUETOOTH_CHECKPOINT_SEND from 'BLUETOOTH', DLC: 1 byte(s), MID: 15
typedef struct {
    uint8_t BLUETOOTH_CHECKPOINT_SEND_cmd; ///< B7:0  Min: 0 Max: 3   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_t;


/// Message: BLUETOOTH_CHECKPOINT_DATA from 'BLUETOOTH', DLC: 1 byte(s), MID: 16
typedef struct {
    uint8_t BLUETOOTH_CHECKPOINT_DATA_cmd; ///< B7:0  Min: 0 Max: 3   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_t;


/// Message: DRIVER_LOC_UPDATE from 'DRIVER', DLC: 8 byte(s), MID: 17
typedef struct {
    float DRIVER_LOC_UPDATE_LAT_cmd;     ///< B31:0   Destination: GEO,MOTORIO
    float DRIVER_LOC_UPDATE_LONG_cmd;    ///< B63:32   Destination: GEO,MOTORIO

    mia_info_t mia_info;
} DRIVER_TX_DRIVER_LOC_UPDATE_t;


/// Message: GEO_SPEED_ANGLE from 'GEO', DLC: 5 byte(s), MID: 18
typedef struct {
    uint8_t GEO_SPEED_cmd;               ///< B7:0   Destination: DRIVER,MOTORIO
    uint16_t GEO_ANGLE_heading_cmd;      ///< B23:8   Destination: DRIVER,MOTORIO
    uint16_t GEO_ANGLE_bearing_cmd;      ///< B39:24   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} GEO_TX_GEO_SPEED_ANGLE_t;


/// Message: GEO_LOC_DATA from 'GEO', DLC: 8 byte(s), MID: 19
typedef struct {
    float GEO_LOC_LAT_cmd;               ///< B31:0   Destination: DRIVER,MOTORIO
    float GEO_LOC_LONG_cmd;              ///< B63:32   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} GEO_TX_GEO_LOC_DATA_t;


/// Message: SENSOR_LIGHT_BAT from 'SENSOR', DLC: 2 byte(s), MID: 20
typedef struct {
    uint8_t SENSOR_LIGHT_cmd;            ///< B7:0  Min: 0 Max: 1   Destination: DRIVER,MOTORIO
    uint8_t SENSOR_BAT_cmd;              ///< B15:8  Min: 0 Max: 1   Destination: DRIVER,MOTORIO

    mia_info_t mia_info;
} SENSOR_TX_SENSOR_LIGHT_BAT_t;


/// These 'externs' need to be defined in a source file of your project
extern const uint32_t DRIVER_KILL_SWITCH__MIA_MS;
extern const DRIVER_TX_DRIVER_KILL_SWITCH_t DRIVER_KILL_SWITCH__MIA_MSG;
extern const uint32_t DRIVER_RESET__MIA_MS;
extern const DRIVER_TX_DRIVER_RESET_t DRIVER_RESET__MIA_MSG;
extern const uint32_t DRIVER_SYNC_ACK__MIA_MS;
extern const DRIVER_TX_DRIVER_SYNC_ACK_t DRIVER_SYNC_ACK__MIA_MSG;
extern const uint32_t MOTORIO_RUNMODE__MIA_MS;
extern const DRIVER_TX_MOTORIO_RUNMODE_t MOTORIO_RUNMODE__MIA_MSG;
extern const uint32_t SENSOR_SONARS__MIA_MS;
extern const SENSOR_TX_SENSOR_SONARS_t SENSOR_SONARS__MIA_MSG;
extern const uint32_t MOTORIO_DIRECTION__MIA_MS;
extern const DRIVER_TX_MOTORIO_DIRECTION_t MOTORIO_DIRECTION__MIA_MSG;
extern const uint32_t DRIVER_CHECKPOINT_REQ__MIA_MS;
extern const DRIVER_TX_DRIVER_CHECKPOINT_REQ_t DRIVER_CHECKPOINT_REQ__MIA_MSG;
extern const uint32_t BLUETOOTH_CHECKPOINT_SEND__MIA_MS;
extern const BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_t BLUETOOTH_CHECKPOINT_SEND__MIA_MSG;
extern const uint32_t BLUETOOTH_CHECKPOINT_DATA__MIA_MS;
extern const BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_t BLUETOOTH_CHECKPOINT_DATA__MIA_MSG;
extern const uint32_t DRIVER_LOC_UPDATE__MIA_MS;
extern const DRIVER_TX_DRIVER_LOC_UPDATE_t DRIVER_LOC_UPDATE__MIA_MSG;
extern const uint32_t GEO_SPEED_ANGLE__MIA_MS;
extern const GEO_TX_GEO_SPEED_ANGLE_t GEO_SPEED_ANGLE__MIA_MSG;
extern const uint32_t GEO_LOC_DATA__MIA_MS;
extern const GEO_TX_GEO_LOC_DATA_t GEO_LOC_DATA__MIA_MSG;
extern const uint32_t SENSOR_LIGHT_BAT__MIA_MS;
extern const SENSOR_TX_SENSOR_LIGHT_BAT_t SENSOR_LIGHT_BAT__MIA_MSG;

/// Not generating code for DRIVER_TX_DRIVER_KILL_SWITCH_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for DRIVER_TX_DRIVER_RESET_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for DRIVER_TX_DRIVER_SYNC_ACK_encode() since the sender is DRIVER and we are MOTORIO

/// Encode MOTORIO's 'MOTORIO_SYNC' message
/// @returns the message header of this message
static msg_hdr_t MOTORIO_TX_MOTORIO_SYNC_encode(uint64_t *to, MOTORIO_TX_MOTORIO_SYNC_t *from)
{
    *to = 0; ///< Default the entire destination data with zeroes
    uint8_t *bytes = (uint8_t*) to;
    uint64_t raw_signal;

    raw_signal = ((uint64_t)(((from->MOTORIO_SYNC_cmd - (0)) / 1.0) + 0.5)) & 0x00;

    return MOTORIO_TX_MOTORIO_SYNC_HDR;
}


/// Not generating code for SENSOR_TX_SENSOR_SYNC_encode() since the sender is SENSOR and we are MOTORIO

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_SYNC_encode() since the sender is BLUETOOTH and we are MOTORIO

/// Not generating code for GEO_TX_GEO_SYNC_encode() since the sender is GEO and we are MOTORIO

/// Encode MOTORIO's 'MOTORIO_HEARTBEAT' message
/// @returns the message header of this message
static msg_hdr_t MOTORIO_TX_MOTORIO_HEARTBEAT_encode(uint64_t *to, MOTORIO_TX_MOTORIO_HEARTBEAT_t *from)
{
    *to = 0; ///< Default the entire destination data with zeroes
    uint8_t *bytes = (uint8_t*) to;
    uint64_t raw_signal;

    raw_signal = ((uint64_t)(((from->MOTORIO_HEARTBEAT_cmd - (0)) / 1.0) + 0.5)) & 0x00;

    return MOTORIO_TX_MOTORIO_HEARTBEAT_HDR;
}


/// Not generating code for SENSOR_TX_SENSOR_HEARTBEAT_encode() since the sender is SENSOR and we are MOTORIO

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_HEARTBEAT_encode() since the sender is BLUETOOTH and we are MOTORIO

/// Not generating code for GEO_TX_GEO_HEARTBEAT_encode() since the sender is GEO and we are MOTORIO

/// Not generating code for DRIVER_TX_MOTORIO_RUNMODE_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for SENSOR_TX_SENSOR_SONARS_encode() since the sender is SENSOR and we are MOTORIO

/// Not generating code for DRIVER_TX_MOTORIO_DIRECTION_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for DRIVER_TX_DRIVER_CHECKPOINT_REQ_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_encode() since the sender is BLUETOOTH and we are MOTORIO

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_encode() since the sender is BLUETOOTH and we are MOTORIO

/// Not generating code for DRIVER_TX_DRIVER_LOC_UPDATE_encode() since the sender is DRIVER and we are MOTORIO

/// Not generating code for GEO_TX_GEO_SPEED_ANGLE_encode() since the sender is GEO and we are MOTORIO

/// Not generating code for GEO_TX_GEO_LOC_DATA_encode() since the sender is GEO and we are MOTORIO

/// Not generating code for SENSOR_TX_SENSOR_LIGHT_BAT_encode() since the sender is SENSOR and we are MOTORIO

/// Decode DRIVER's 'DRIVER_KILL_SWITCH' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_DRIVER_KILL_SWITCH_decode(DRIVER_TX_DRIVER_KILL_SWITCH_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_DRIVER_KILL_SWITCH_HDR.dlc || hdr->mid != DRIVER_TX_DRIVER_KILL_SWITCH_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    to->DRIVER_KILL_SWITCH_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode DRIVER's 'DRIVER_RESET' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_DRIVER_RESET_decode(DRIVER_TX_DRIVER_RESET_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_DRIVER_RESET_HDR.dlc || hdr->mid != DRIVER_TX_DRIVER_RESET_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    to->DRIVER_RESET_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode DRIVER's 'DRIVER_SYNC_ACK' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_DRIVER_SYNC_ACK_decode(DRIVER_TX_DRIVER_SYNC_ACK_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_DRIVER_SYNC_ACK_HDR.dlc || hdr->mid != DRIVER_TX_DRIVER_SYNC_ACK_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    to->DRIVER_SYNC_ACK_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Not generating code for MOTORIO_TX_MOTORIO_SYNC_decode() since we are not the recipient of any of its signals

/// Not generating code for SENSOR_TX_SENSOR_SYNC_decode() since we are not the recipient of any of its signals

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_SYNC_decode() since we are not the recipient of any of its signals

/// Not generating code for GEO_TX_GEO_SYNC_decode() since we are not the recipient of any of its signals

/// Not generating code for MOTORIO_TX_MOTORIO_HEARTBEAT_decode() since we are not the recipient of any of its signals

/// Not generating code for SENSOR_TX_SENSOR_HEARTBEAT_decode() since we are not the recipient of any of its signals

/// Not generating code for BLUETOOTH_TX_BLUETOOTH_HEARTBEAT_decode() since we are not the recipient of any of its signals

/// Not generating code for GEO_TX_GEO_HEARTBEAT_decode() since we are not the recipient of any of its signals

/// Decode DRIVER's 'MOTORIO_RUNMODE' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_MOTORIO_RUNMODE_decode(DRIVER_TX_MOTORIO_RUNMODE_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_MOTORIO_RUNMODE_HDR.dlc || hdr->mid != DRIVER_TX_MOTORIO_RUNMODE_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->MOTORIO_RUNMODE_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode SENSOR's 'SENSOR_SONARS' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool SENSOR_TX_SENSOR_SONARS_decode(SENSOR_TX_SENSOR_SONARS_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != SENSOR_TX_SENSOR_SONARS_HDR.dlc || hdr->mid != SENSOR_TX_SENSOR_SONARS_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_front_left = (raw_signal * 1.0) + (0);
    to->SENSOR_SONARS_front_left = bytes[0];

    raw_signal = 0;
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_front_right = (raw_signal * 1.0) + (0);
    to->SENSOR_SONARS_front_right = bytes[1];

    raw_signal = 0;
    bits_from_byte = ((bytes[2] >> 0) & 0xff); ///< 8 bit(s) from B16
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_front_center = (raw_signal * 1.0) + (0);
    to->SENSOR_SONARS_front_center = bytes[2];

    raw_signal = 0;
    bits_from_byte = ((bytes[3] >> 0) & 0xff); ///< 8 bit(s) from B24
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_left = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[4] >> 0) & 0xff); ///< 8 bit(s) from B32
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_right = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[5] >> 0) & 0xff); ///< 8 bit(s) from B40
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_SONARS_back = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode DRIVER's 'MOTORIO_DIRECTION' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_MOTORIO_DIRECTION_decode(DRIVER_TX_MOTORIO_DIRECTION_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_MOTORIO_DIRECTION_HDR.dlc || hdr->mid != DRIVER_TX_MOTORIO_DIRECTION_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->MOTORIO_DIRECTION_speed_cmd = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 0);
    to->MOTORIO_DIRECTION_turn_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode DRIVER's 'DRIVER_CHECKPOINT_REQ' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_DRIVER_CHECKPOINT_REQ_decode(DRIVER_TX_DRIVER_CHECKPOINT_REQ_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_DRIVER_CHECKPOINT_REQ_HDR.dlc || hdr->mid != DRIVER_TX_DRIVER_CHECKPOINT_REQ_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->DRIVER_CHECKPOINT_REQ_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode BLUETOOTH's 'BLUETOOTH_CHECKPOINT_SEND' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_decode(BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_HDR.dlc || hdr->mid != BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->BLUETOOTH_CHECKPOINT_SEND_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode BLUETOOTH's 'BLUETOOTH_CHECKPOINT_DATA' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_decode(BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_HDR.dlc || hdr->mid != BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->BLUETOOTH_CHECKPOINT_DATA_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode DRIVER's 'DRIVER_LOC_UPDATE' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool DRIVER_TX_DRIVER_LOC_UPDATE_decode(DRIVER_TX_DRIVER_LOC_UPDATE_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != DRIVER_TX_DRIVER_LOC_UPDATE_HDR.dlc || hdr->mid != DRIVER_TX_DRIVER_LOC_UPDATE_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 8);
    bits_from_byte = ((bytes[2] >> 0) & 0xff); ///< 8 bit(s) from B16
    raw_signal    |= (bits_from_byte << 16);
    bits_from_byte = ((bytes[3] >> 0) & 0xff); ///< 8 bit(s) from B24
    raw_signal    |= (bits_from_byte << 24);
    to->DRIVER_LOC_UPDATE_LAT_cmd = (raw_signal * 0.0001) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[4] >> 0) & 0xff); ///< 8 bit(s) from B32
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[5] >> 0) & 0xff); ///< 8 bit(s) from B40
    raw_signal    |= (bits_from_byte << 8);
    bits_from_byte = ((bytes[6] >> 0) & 0xff); ///< 8 bit(s) from B48
    raw_signal    |= (bits_from_byte << 16);
    bits_from_byte = ((bytes[7] >> 0) & 0xff); ///< 8 bit(s) from B56
    raw_signal    |= (bits_from_byte << 24);
    to->DRIVER_LOC_UPDATE_LONG_cmd = (raw_signal * 0.0001) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode GEO's 'GEO_SPEED_ANGLE' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool GEO_TX_GEO_SPEED_ANGLE_decode(GEO_TX_GEO_SPEED_ANGLE_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != GEO_TX_GEO_SPEED_ANGLE_HDR.dlc || hdr->mid != GEO_TX_GEO_SPEED_ANGLE_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->GEO_SPEED_cmd = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[2] >> 0) & 0xff); ///< 8 bit(s) from B16
    raw_signal    |= (bits_from_byte << 8);
    to->GEO_ANGLE_heading_cmd = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[3] >> 0) & 0xff); ///< 8 bit(s) from B24
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[4] >> 0) & 0xff); ///< 8 bit(s) from B32
    raw_signal    |= (bits_from_byte << 8);
    to->GEO_ANGLE_bearing_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode GEO's 'GEO_LOC_DATA' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool GEO_TX_GEO_LOC_DATA_decode(GEO_TX_GEO_LOC_DATA_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != GEO_TX_GEO_LOC_DATA_HDR.dlc || hdr->mid != GEO_TX_GEO_LOC_DATA_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 8);
    bits_from_byte = ((bytes[2] >> 0) & 0xff); ///< 8 bit(s) from B16
    raw_signal    |= (bits_from_byte << 16);
    bits_from_byte = ((bytes[3] >> 0) & 0xff); ///< 8 bit(s) from B24
    raw_signal    |= (bits_from_byte << 24);
    to->GEO_LOC_LAT_cmd = (raw_signal * 0.0001) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[4] >> 0) & 0xff); ///< 8 bit(s) from B32
    raw_signal    |= (bits_from_byte << 0);
    bits_from_byte = ((bytes[5] >> 0) & 0xff); ///< 8 bit(s) from B40
    raw_signal    |= (bits_from_byte << 8);
    bits_from_byte = ((bytes[6] >> 0) & 0xff); ///< 8 bit(s) from B48
    raw_signal    |= (bits_from_byte << 16);
    bits_from_byte = ((bytes[7] >> 0) & 0xff); ///< 8 bit(s) from B56
    raw_signal    |= (bits_from_byte << 24);
    to->GEO_LOC_LONG_cmd = (raw_signal * 0.0001) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Decode SENSOR's 'SENSOR_LIGHT_BAT' message
/// @param hdr  The header of the message to validate its DLC and MID; this can be NULL to skip this check
static inline bool SENSOR_TX_SENSOR_LIGHT_BAT_decode(SENSOR_TX_SENSOR_LIGHT_BAT_t *to, const uint64_t *from, const msg_hdr_t *hdr)
{
    const bool success = true;
    if (NULL != hdr && (hdr->dlc != SENSOR_TX_SENSOR_LIGHT_BAT_HDR.dlc || hdr->mid != SENSOR_TX_SENSOR_LIGHT_BAT_HDR.mid)) {
        return !success;
    }
    uint64_t raw_signal;
    uint64_t bits_from_byte;
    const uint8_t *bytes = (const uint8_t*) from;

    raw_signal = 0;
    bits_from_byte = ((bytes[0] >> 0) & 0xff); ///< 8 bit(s) from B0
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_LIGHT_cmd = (raw_signal * 1.0) + (0);

    raw_signal = 0;
    bits_from_byte = ((bytes[1] >> 0) & 0xff); ///< 8 bit(s) from B8
    raw_signal    |= (bits_from_byte << 0);
    to->SENSOR_BAT_cmd = (raw_signal * 1.0) + (0);

    to->mia_info.mia_counter_ms = 0; ///< Reset the MIA counter
    return success;
}


/// Handle the MIA for DRIVER's 'DRIVER_KILL_SWITCH' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_DRIVER_KILL_SWITCH_handle_mia(DRIVER_TX_DRIVER_KILL_SWITCH_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= DRIVER_KILL_SWITCH__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = DRIVER_KILL_SWITCH__MIA_MSG;
        msg->mia_info.mia_counter_ms = DRIVER_KILL_SWITCH__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'DRIVER_RESET' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_DRIVER_RESET_handle_mia(DRIVER_TX_DRIVER_RESET_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= DRIVER_RESET__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = DRIVER_RESET__MIA_MSG;
        msg->mia_info.mia_counter_ms = DRIVER_RESET__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'DRIVER_SYNC_ACK' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_DRIVER_SYNC_ACK_handle_mia(DRIVER_TX_DRIVER_SYNC_ACK_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= DRIVER_SYNC_ACK__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = DRIVER_SYNC_ACK__MIA_MSG;
        msg->mia_info.mia_counter_ms = DRIVER_SYNC_ACK__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'MOTORIO_RUNMODE' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_MOTORIO_RUNMODE_handle_mia(DRIVER_TX_MOTORIO_RUNMODE_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= MOTORIO_RUNMODE__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = MOTORIO_RUNMODE__MIA_MSG;
        msg->mia_info.mia_counter_ms = MOTORIO_RUNMODE__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for SENSOR's 'SENSOR_SONARS' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool SENSOR_TX_SENSOR_SONARS_handle_mia(SENSOR_TX_SENSOR_SONARS_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= SENSOR_SONARS__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = SENSOR_SONARS__MIA_MSG;
        msg->mia_info.mia_counter_ms = SENSOR_SONARS__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'MOTORIO_DIRECTION' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_MOTORIO_DIRECTION_handle_mia(DRIVER_TX_MOTORIO_DIRECTION_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= MOTORIO_DIRECTION__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = MOTORIO_DIRECTION__MIA_MSG;
        msg->mia_info.mia_counter_ms = MOTORIO_DIRECTION__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'DRIVER_CHECKPOINT_REQ' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_DRIVER_CHECKPOINT_REQ_handle_mia(DRIVER_TX_DRIVER_CHECKPOINT_REQ_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= DRIVER_CHECKPOINT_REQ__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = DRIVER_CHECKPOINT_REQ__MIA_MSG;
        msg->mia_info.mia_counter_ms = DRIVER_CHECKPOINT_REQ__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for BLUETOOTH's 'BLUETOOTH_CHECKPOINT_SEND' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_handle_mia(BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_SEND_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= BLUETOOTH_CHECKPOINT_SEND__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = BLUETOOTH_CHECKPOINT_SEND__MIA_MSG;
        msg->mia_info.mia_counter_ms = BLUETOOTH_CHECKPOINT_SEND__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for BLUETOOTH's 'BLUETOOTH_CHECKPOINT_DATA' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_handle_mia(BLUETOOTH_TX_BLUETOOTH_CHECKPOINT_DATA_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= BLUETOOTH_CHECKPOINT_DATA__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = BLUETOOTH_CHECKPOINT_DATA__MIA_MSG;
        msg->mia_info.mia_counter_ms = BLUETOOTH_CHECKPOINT_DATA__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for DRIVER's 'DRIVER_LOC_UPDATE' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool DRIVER_TX_DRIVER_LOC_UPDATE_handle_mia(DRIVER_TX_DRIVER_LOC_UPDATE_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= DRIVER_LOC_UPDATE__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = DRIVER_LOC_UPDATE__MIA_MSG;
        msg->mia_info.mia_counter_ms = DRIVER_LOC_UPDATE__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for GEO's 'GEO_SPEED_ANGLE' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool GEO_TX_GEO_SPEED_ANGLE_handle_mia(GEO_TX_GEO_SPEED_ANGLE_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= GEO_SPEED_ANGLE__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = GEO_SPEED_ANGLE__MIA_MSG;
        msg->mia_info.mia_counter_ms = GEO_SPEED_ANGLE__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for GEO's 'GEO_LOC_DATA' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool GEO_TX_GEO_LOC_DATA_handle_mia(GEO_TX_GEO_LOC_DATA_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= GEO_LOC_DATA__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = GEO_LOC_DATA__MIA_MSG;
        msg->mia_info.mia_counter_ms = GEO_LOC_DATA__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}

/// Handle the MIA for SENSOR's 'SENSOR_LIGHT_BAT' message
/// @param   time_incr_ms  The time to increment the MIA counter with
/// @returns true if the MIA just occurred
/// @post    If the MIA counter is not reset, and goes beyond the MIA value, the MIA flag is set
static inline bool SENSOR_TX_SENSOR_LIGHT_BAT_handle_mia(SENSOR_TX_SENSOR_LIGHT_BAT_t *msg, uint32_t time_incr_ms)
{
    bool mia_occurred = false;
    const mia_info_t old_mia = msg->mia_info;
    msg->mia_info.is_mia = (msg->mia_info.mia_counter_ms >= SENSOR_LIGHT_BAT__MIA_MS);

    if (!msg->mia_info.is_mia) { 
        msg->mia_info.mia_counter_ms += time_incr_ms;
    }
    else if(!old_mia.is_mia)   { 
        // Copy MIA struct, then re-write the MIA counter and is_mia that is overwriten
        *msg = SENSOR_LIGHT_BAT__MIA_MSG;
        msg->mia_info.mia_counter_ms = SENSOR_LIGHT_BAT__MIA_MS;
        msg->mia_info.is_mia = true;
        mia_occurred = true;
    }

    return mia_occurred;
}
