/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_SCHEMA_PB_H_INCLUDED
#define PB_SCHEMA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum flight_state {
    FLIGHT_STATE_IDLE = 0,
    FLIGHT_STATE_LAUNCHPAD = 1,
    FLIGHT_STATE_BOOST = 2,
    FLIGHT_STATE_COAST = 3,
    FLIGHT_STATE_FREE_FALL = 4,
    FLIGHT_STATE_DROGUE_DESCENT = 5,
    FLIGHT_STATE_MAIN_DESCENT = 6,
    FLIGHT_STATE_LANDED = 7
} flight_state_t;

/* Struct definitions */
typedef struct acknowledge {
    bool success;
} acknowledge_t;

typedef struct pressure_reading {
    float pressure;
} pressure_reading_t;

typedef struct imu_reading {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} imu_reading_t;

typedef struct telemetry_packet {
    float altitude;
    float longitude;
    float latitude;
    bool pyro0_connected;
    bool pyro1_connected;
    bool pyro2_connected;
    flight_state_t flight_state;
    float az;
    float velocity;
    float battery;
    int32_t flash_address;
} telemetry_packet_t;

typedef struct gnss_position {
    float longitude;
    float latitude;
} gnss_position_t;

typedef struct gnss_status {
    int32_t fix;
    int32_t num_satellites;
    float hdop;
    float vdop;
} gnss_status_t;

typedef struct set_sudo {
    bool enabled;
} set_sudo_t;

typedef struct clear_flash {
    char dummy_field;
} clear_flash_t;

typedef struct read_flash {
    int32_t start_index;
    int32_t length;
} read_flash_t;

typedef PB_BYTES_ARRAY_T(64) flash_data_data_t;
typedef struct flash_data {
    int32_t start_index;
    flash_data_data_t data;
} flash_data_t;

typedef struct ready_up {
    char dummy_field;
} ready_up_t;

typedef struct enter_idle {
    char dummy_field;
} enter_idle_t;

typedef struct pyros_enabled {
    bool pyro0;
    bool pyro1;
    bool pyro2;
} pyros_enabled_t;

typedef struct enable_pyros {
    bool pyro0;
    bool pyro1;
    bool pyro2;
} enable_pyros_t;

typedef struct fjalar_data {
    pb_size_t which_data;
    union {
        acknowledge_t acknowledge;
        telemetry_packet_t telemetry_packet;
        imu_reading_t imu_reading;
        pressure_reading_t pressure_reading;
        gnss_position_t gnss_position;
        gnss_status_t gnss_status;
        set_sudo_t set_sudo;
        clear_flash_t clear_flash;
        read_flash_t read_flash;
        flash_data_t flash_data;
        ready_up_t ready_up;
        enter_idle_t enter_idle;
        enable_pyros_t enable_pyros;
        pyros_enabled_t pyros_enabled;
    } data;
} fjalar_data_t;

typedef struct fjalar_message {
    uint32_t time;
    int32_t sequence_number;
    bool has_data;
    fjalar_data_t data;
} fjalar_message_t;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _FLIGHT_STATE_MIN FLIGHT_STATE_IDLE
#define _FLIGHT_STATE_MAX FLIGHT_STATE_LANDED
#define _FLIGHT_STATE_ARRAYSIZE ((flight_state_t)(FLIGHT_STATE_LANDED+1))




#define telemetry_packet_t_flight_state_ENUMTYPE flight_state_t














/* Initializer values for message structs */
#define ACKNOWLEDGE_INIT_DEFAULT                 {0}
#define PRESSURE_READING_INIT_DEFAULT            {0}
#define IMU_READING_INIT_DEFAULT                 {0, 0, 0, 0, 0, 0}
#define TELEMETRY_PACKET_INIT_DEFAULT            {0, 0, 0, 0, 0, 0, _FLIGHT_STATE_MIN, 0, 0, 0, 0}
#define GNSS_POSITION_INIT_DEFAULT               {0, 0}
#define GNSS_STATUS_INIT_DEFAULT                 {0, 0, 0, 0}
#define SET_SUDO_INIT_DEFAULT                    {0}
#define CLEAR_FLASH_INIT_DEFAULT                 {0}
#define READ_FLASH_INIT_DEFAULT                  {0, 0}
#define FLASH_DATA_INIT_DEFAULT                  {0, {0, {0}}}
#define READY_UP_INIT_DEFAULT                    {0}
#define ENTER_IDLE_INIT_DEFAULT                  {0}
#define PYROS_ENABLED_INIT_DEFAULT               {0, 0, 0}
#define ENABLE_PYROS_INIT_DEFAULT                {0, 0, 0}
#define FJALAR_DATA_INIT_DEFAULT                 {0, {ACKNOWLEDGE_INIT_DEFAULT}}
#define FJALAR_MESSAGE_INIT_DEFAULT              {0, 0, false, FJALAR_DATA_INIT_DEFAULT}
#define ACKNOWLEDGE_INIT_ZERO                    {0}
#define PRESSURE_READING_INIT_ZERO               {0}
#define IMU_READING_INIT_ZERO                    {0, 0, 0, 0, 0, 0}
#define TELEMETRY_PACKET_INIT_ZERO               {0, 0, 0, 0, 0, 0, _FLIGHT_STATE_MIN, 0, 0, 0, 0}
#define GNSS_POSITION_INIT_ZERO                  {0, 0}
#define GNSS_STATUS_INIT_ZERO                    {0, 0, 0, 0}
#define SET_SUDO_INIT_ZERO                       {0}
#define CLEAR_FLASH_INIT_ZERO                    {0}
#define READ_FLASH_INIT_ZERO                     {0, 0}
#define FLASH_DATA_INIT_ZERO                     {0, {0, {0}}}
#define READY_UP_INIT_ZERO                       {0}
#define ENTER_IDLE_INIT_ZERO                     {0}
#define PYROS_ENABLED_INIT_ZERO                  {0, 0, 0}
#define ENABLE_PYROS_INIT_ZERO                   {0, 0, 0}
#define FJALAR_DATA_INIT_ZERO                    {0, {ACKNOWLEDGE_INIT_ZERO}}
#define FJALAR_MESSAGE_INIT_ZERO                 {0, 0, false, FJALAR_DATA_INIT_ZERO}

/* Field tags (for use in manual encoding/decoding) */
#define ACKNOWLEDGE_SUCCESS_TAG                  1
#define PRESSURE_READING_PRESSURE_TAG            1
#define IMU_READING_AX_TAG                       1
#define IMU_READING_AY_TAG                       2
#define IMU_READING_AZ_TAG                       3
#define IMU_READING_GX_TAG                       4
#define IMU_READING_GY_TAG                       5
#define IMU_READING_GZ_TAG                       6
#define TELEMETRY_PACKET_ALTITUDE_TAG            1
#define TELEMETRY_PACKET_LONGITUDE_TAG           2
#define TELEMETRY_PACKET_LATITUDE_TAG            3
#define TELEMETRY_PACKET_PYRO0_CONNECTED_TAG     4
#define TELEMETRY_PACKET_PYRO1_CONNECTED_TAG     5
#define TELEMETRY_PACKET_PYRO2_CONNECTED_TAG     6
#define TELEMETRY_PACKET_FLIGHT_STATE_TAG        7
#define TELEMETRY_PACKET_AZ_TAG                  10
#define TELEMETRY_PACKET_VELOCITY_TAG            11
#define TELEMETRY_PACKET_BATTERY_TAG             12
#define TELEMETRY_PACKET_FLASH_ADDRESS_TAG       13
#define GNSS_POSITION_LONGITUDE_TAG              1
#define GNSS_POSITION_LATITUDE_TAG               2
#define GNSS_STATUS_FIX_TAG                      1
#define GNSS_STATUS_NUM_SATELLITES_TAG           2
#define GNSS_STATUS_HDOP_TAG                     3
#define GNSS_STATUS_VDOP_TAG                     4
#define SET_SUDO_ENABLED_TAG                     1
#define READ_FLASH_START_INDEX_TAG               1
#define READ_FLASH_LENGTH_TAG                    2
#define FLASH_DATA_START_INDEX_TAG               1
#define FLASH_DATA_DATA_TAG                      2
#define PYROS_ENABLED_PYRO0_TAG                  1
#define PYROS_ENABLED_PYRO1_TAG                  2
#define PYROS_ENABLED_PYRO2_TAG                  3
#define ENABLE_PYROS_PYRO0_TAG                   1
#define ENABLE_PYROS_PYRO1_TAG                   2
#define ENABLE_PYROS_PYRO2_TAG                   3
#define FJALAR_DATA_ACKNOWLEDGE_TAG              1
#define FJALAR_DATA_TELEMETRY_PACKET_TAG         2
#define FJALAR_DATA_IMU_READING_TAG              3
#define FJALAR_DATA_PRESSURE_READING_TAG         4
#define FJALAR_DATA_GNSS_POSITION_TAG            5
#define FJALAR_DATA_GNSS_STATUS_TAG              6
#define FJALAR_DATA_SET_SUDO_TAG                 8
#define FJALAR_DATA_CLEAR_FLASH_TAG              9
#define FJALAR_DATA_READ_FLASH_TAG               10
#define FJALAR_DATA_FLASH_DATA_TAG               11
#define FJALAR_DATA_READY_UP_TAG                 12
#define FJALAR_DATA_ENTER_IDLE_TAG               13
#define FJALAR_DATA_ENABLE_PYROS_TAG             14
#define FJALAR_DATA_PYROS_ENABLED_TAG            15
#define FJALAR_MESSAGE_TIME_TAG                  1
#define FJALAR_MESSAGE_SEQUENCE_NUMBER_TAG       2
#define FJALAR_MESSAGE_DATA_TAG                  3

/* Struct field encoding specification for nanopb */
#define ACKNOWLEDGE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     success,           1)
#define ACKNOWLEDGE_CALLBACK NULL
#define ACKNOWLEDGE_DEFAULT NULL

#define PRESSURE_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    pressure,          1)
#define PRESSURE_READING_CALLBACK NULL
#define PRESSURE_READING_DEFAULT NULL

#define IMU_READING_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    ax,                1) \
X(a, STATIC,   SINGULAR, FLOAT,    ay,                2) \
X(a, STATIC,   SINGULAR, FLOAT,    az,                3) \
X(a, STATIC,   SINGULAR, FLOAT,    gx,                4) \
X(a, STATIC,   SINGULAR, FLOAT,    gy,                5) \
X(a, STATIC,   SINGULAR, FLOAT,    gz,                6)
#define IMU_READING_CALLBACK NULL
#define IMU_READING_DEFAULT NULL

#define TELEMETRY_PACKET_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    altitude,          1) \
X(a, STATIC,   SINGULAR, FLOAT,    longitude,         2) \
X(a, STATIC,   SINGULAR, FLOAT,    latitude,          3) \
X(a, STATIC,   SINGULAR, BOOL,     pyro0_connected,   4) \
X(a, STATIC,   SINGULAR, BOOL,     pyro1_connected,   5) \
X(a, STATIC,   SINGULAR, BOOL,     pyro2_connected,   6) \
X(a, STATIC,   SINGULAR, UENUM,    flight_state,      7) \
X(a, STATIC,   SINGULAR, FLOAT,    az,               10) \
X(a, STATIC,   SINGULAR, FLOAT,    velocity,         11) \
X(a, STATIC,   SINGULAR, FLOAT,    battery,          12) \
X(a, STATIC,   SINGULAR, INT32,    flash_address,    13)
#define TELEMETRY_PACKET_CALLBACK NULL
#define TELEMETRY_PACKET_DEFAULT NULL

#define GNSS_POSITION_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    longitude,         1) \
X(a, STATIC,   SINGULAR, FLOAT,    latitude,          2)
#define GNSS_POSITION_CALLBACK NULL
#define GNSS_POSITION_DEFAULT NULL

#define GNSS_STATUS_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    fix,               1) \
X(a, STATIC,   SINGULAR, INT32,    num_satellites,    2) \
X(a, STATIC,   SINGULAR, FLOAT,    hdop,              3) \
X(a, STATIC,   SINGULAR, FLOAT,    vdop,              4)
#define GNSS_STATUS_CALLBACK NULL
#define GNSS_STATUS_DEFAULT NULL

#define SET_SUDO_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     enabled,           1)
#define SET_SUDO_CALLBACK NULL
#define SET_SUDO_DEFAULT NULL

#define CLEAR_FLASH_FIELDLIST(X, a) \

#define CLEAR_FLASH_CALLBACK NULL
#define CLEAR_FLASH_DEFAULT NULL

#define READ_FLASH_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    start_index,       1) \
X(a, STATIC,   SINGULAR, INT32,    length,            2)
#define READ_FLASH_CALLBACK NULL
#define READ_FLASH_DEFAULT NULL

#define FLASH_DATA_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT32,    start_index,       1) \
X(a, STATIC,   SINGULAR, BYTES,    data,              2)
#define FLASH_DATA_CALLBACK NULL
#define FLASH_DATA_DEFAULT NULL

#define READY_UP_FIELDLIST(X, a) \

#define READY_UP_CALLBACK NULL
#define READY_UP_DEFAULT NULL

#define ENTER_IDLE_FIELDLIST(X, a) \

#define ENTER_IDLE_CALLBACK NULL
#define ENTER_IDLE_DEFAULT NULL

#define PYROS_ENABLED_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     pyro0,             1) \
X(a, STATIC,   SINGULAR, BOOL,     pyro1,             2) \
X(a, STATIC,   SINGULAR, BOOL,     pyro2,             3)
#define PYROS_ENABLED_CALLBACK NULL
#define PYROS_ENABLED_DEFAULT NULL

#define ENABLE_PYROS_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     pyro0,             1) \
X(a, STATIC,   SINGULAR, BOOL,     pyro1,             2) \
X(a, STATIC,   SINGULAR, BOOL,     pyro2,             3)
#define ENABLE_PYROS_CALLBACK NULL
#define ENABLE_PYROS_DEFAULT NULL

#define FJALAR_DATA_FIELDLIST(X, a) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,acknowledge,data.acknowledge),   1) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,telemetry_packet,data.telemetry_packet),   2) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,imu_reading,data.imu_reading),   3) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,pressure_reading,data.pressure_reading),   4) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,gnss_position,data.gnss_position),   5) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,gnss_status,data.gnss_status),   6) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,set_sudo,data.set_sudo),   8) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,clear_flash,data.clear_flash),   9) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,read_flash,data.read_flash),  10) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,flash_data,data.flash_data),  11) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,ready_up,data.ready_up),  12) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,enter_idle,data.enter_idle),  13) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,enable_pyros,data.enable_pyros),  14) \
X(a, STATIC,   ONEOF,    MESSAGE,  (data,pyros_enabled,data.pyros_enabled),  15)
#define FJALAR_DATA_CALLBACK NULL
#define FJALAR_DATA_DEFAULT NULL
#define fjalar_data_t_data_acknowledge_MSGTYPE acknowledge_t
#define fjalar_data_t_data_telemetry_packet_MSGTYPE telemetry_packet_t
#define fjalar_data_t_data_imu_reading_MSGTYPE imu_reading_t
#define fjalar_data_t_data_pressure_reading_MSGTYPE pressure_reading_t
#define fjalar_data_t_data_gnss_position_MSGTYPE gnss_position_t
#define fjalar_data_t_data_gnss_status_MSGTYPE gnss_status_t
#define fjalar_data_t_data_set_sudo_MSGTYPE set_sudo_t
#define fjalar_data_t_data_clear_flash_MSGTYPE clear_flash_t
#define fjalar_data_t_data_read_flash_MSGTYPE read_flash_t
#define fjalar_data_t_data_flash_data_MSGTYPE flash_data_t
#define fjalar_data_t_data_ready_up_MSGTYPE ready_up_t
#define fjalar_data_t_data_enter_idle_MSGTYPE enter_idle_t
#define fjalar_data_t_data_enable_pyros_MSGTYPE enable_pyros_t
#define fjalar_data_t_data_pyros_enabled_MSGTYPE pyros_enabled_t

#define FJALAR_MESSAGE_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FIXED32,  time,              1) \
X(a, STATIC,   SINGULAR, INT32,    sequence_number,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  data,              3)
#define FJALAR_MESSAGE_CALLBACK NULL
#define FJALAR_MESSAGE_DEFAULT NULL
#define fjalar_message_t_data_MSGTYPE fjalar_data_t

extern const pb_msgdesc_t acknowledge_t_msg;
extern const pb_msgdesc_t pressure_reading_t_msg;
extern const pb_msgdesc_t imu_reading_t_msg;
extern const pb_msgdesc_t telemetry_packet_t_msg;
extern const pb_msgdesc_t gnss_position_t_msg;
extern const pb_msgdesc_t gnss_status_t_msg;
extern const pb_msgdesc_t set_sudo_t_msg;
extern const pb_msgdesc_t clear_flash_t_msg;
extern const pb_msgdesc_t read_flash_t_msg;
extern const pb_msgdesc_t flash_data_t_msg;
extern const pb_msgdesc_t ready_up_t_msg;
extern const pb_msgdesc_t enter_idle_t_msg;
extern const pb_msgdesc_t pyros_enabled_t_msg;
extern const pb_msgdesc_t enable_pyros_t_msg;
extern const pb_msgdesc_t fjalar_data_t_msg;
extern const pb_msgdesc_t fjalar_message_t_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define ACKNOWLEDGE_FIELDS &acknowledge_t_msg
#define PRESSURE_READING_FIELDS &pressure_reading_t_msg
#define IMU_READING_FIELDS &imu_reading_t_msg
#define TELEMETRY_PACKET_FIELDS &telemetry_packet_t_msg
#define GNSS_POSITION_FIELDS &gnss_position_t_msg
#define GNSS_STATUS_FIELDS &gnss_status_t_msg
#define SET_SUDO_FIELDS &set_sudo_t_msg
#define CLEAR_FLASH_FIELDS &clear_flash_t_msg
#define READ_FLASH_FIELDS &read_flash_t_msg
#define FLASH_DATA_FIELDS &flash_data_t_msg
#define READY_UP_FIELDS &ready_up_t_msg
#define ENTER_IDLE_FIELDS &enter_idle_t_msg
#define PYROS_ENABLED_FIELDS &pyros_enabled_t_msg
#define ENABLE_PYROS_FIELDS &enable_pyros_t_msg
#define FJALAR_DATA_FIELDS &fjalar_data_t_msg
#define FJALAR_MESSAGE_FIELDS &fjalar_message_t_msg

/* Maximum encoded size of messages (where known) */
#define ACKNOWLEDGE_SIZE                         2
#define CLEAR_FLASH_SIZE                         0
#define ENABLE_PYROS_SIZE                        6
#define ENTER_IDLE_SIZE                          0
#define FJALAR_DATA_SIZE                         79
#define FJALAR_MESSAGE_SIZE                      97
#define FLASH_DATA_SIZE                          77
#define GNSS_POSITION_SIZE                       10
#define GNSS_STATUS_SIZE                         32
#define IMU_READING_SIZE                         30
#define PRESSURE_READING_SIZE                    5
#define PYROS_ENABLED_SIZE                       6
#define READY_UP_SIZE                            0
#define READ_FLASH_SIZE                          22
#define SET_SUDO_SIZE                            2
#define TELEMETRY_PACKET_SIZE                    49

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif