/*
  ESC Telemetry for XC Technology DroneCan ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once 

#include <AP_HAL/AP_HAL.h>

#ifdef HAL_PERIPH_ENABLE_XCESC

class XCESC_Telem {
public:
    XCESC_Telem();

    void init(AP_HAL::UARTDriver* uart);
    bool updata();

    struct XCESC {
        uint32_t counter;
        uint16_t throt_req;
        uint16_t throt_val;
        uint16_t rmp;
        float voltage;
        float phase_current;
        float current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint32_t error_count;
    };

    const XCESC &get_telem(void) {
        return decoded;
    }

private:
    AP_HAL::UARTDriver* uart;

    struct Packed {
        uint8_t header;
        uint8_t pkt_len;
        uint32_t counter;
        uint16_t throt_req;
        uint16_t throt_req;
        uint16_t rpm;
        uint16_t voltage;
        int16_t current;
        int16_t phase_current;
        uint8_t mos_temperature;
        uint8_t cap_temperature;
        uint16_t status;
        uint16_t crc;
    }pkt;

    uint8_t len;
    uint32_t last_read_ms;
    uint32_t error_count;

    struct XCESC decoded;

    bool parse_packet(void);
    uint8_t temperature_decode(uint8_t temp_raw) const;
};

#endif //HAL_PERIPH_ENABLE_XCESC
