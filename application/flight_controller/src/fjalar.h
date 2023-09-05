#pragma once
#define DT_ALIAS_EXISTS(alias) DT_NODE_EXISTS(DT_ALIAS(alias))

enum fjalar_flight_state {
    STATE_IDLE,
    STATE_LAUNCHPAD,
    STATE_BOOST,
    STATE_COAST,
    STATE_FREE_FALL,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_LANDED,
};

enum fjalar_flight_event {
    EVENT_LAUNCH,
    EVENT_BURNOUT,
    EVENT_APOGEE,
    EVENT_PRIMARY_DEPLOY,
    EVENT_SECONDARY_DEPLOY,
    EVENT_LANDED
};

typedef struct {
    enum fjalar_flight_state flight_state;
    float altitude;
    float ground_level;
    float velocity;
    float ax;
    float ay;
    float az;
    bool drogue_deployed;
    bool main_deployed;
    uint32_t liftoff_at;
    uint32_t apogee_at;
    bool sudo;
    uint32_t flash_address;
    uint32_t flash_size;
} fjalar_t;

extern fjalar_t fjalar_god;