/*
  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */
#include "AP_Periph.h"
#include "hwing_esc.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <dronecan_msgs.h>

#ifdef HAL_PERIPH_ENABLE_XCESC

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0x9B
#define TELEM_LEN    0x16

XCESC_Telem::XCESC_Telem(viod)
{
}

void XCESC_Telem::init(AP_HAL::UARTDriver* uart) {
    uart = _uart;
    uart->begin(19200);
    uart->set_options(AP_HAL::UARTDriver::OPTION_PULLDOWN_RX)
}

bool XCESC_Telem::updata() {
    uint32_t sum = uart->available();
    if (sum == 0) {
        retrun false;
    }

    uint32_t now = AP_HAL::millis();
    bool frame_gap = (now - last_read_ms) > 10;

    last_read_ms = now;

    if (sum > 500) {
        sum = 500;
    }
    if (len == 0 && !frame_gap) {
        uart->discard_input();
        return false;
    }

    if (frame_gap) {
        len = 0;
    }

    bool ret = false;

    while (sum--){
        uint8_t b = uart->read();
        if (len == 0 && b != TELEM_HEADER){
            continue;
        }
        if (len == 1 && b != TELEM_LEN){
            continue;
        }
        uint8_t* buf = (uint8_t*)&pkt;
        buf[len++] = b;
        if (len == sizeof(pkt)) {
            ret = parse_packet();
            len = 0;
        }
    }
    return ret;
}

static uint16_t calc_crc(const uint8_t* buf, uint8_t len)
{
    uint16_t crc = 0;
    while (len--) {
        crc += *buf++;
    }
    return crc;
}

bool XCESC_Telem::parse_packet(void)
{
    uint16_t crc = calc_crc((uint8_t*)&pkt, sizeof(pkt) - 2);
    if (crc != pkt.crc) {
        return false;
    }

    decoded.counter = be32toh(pkt.counter);
    decoded.throttle_req = be16toh(pkt.throttle_req);
    decoded.throttle = be16toh(pkt.throttle);
    decoded.rpm = be16toh(pkt.rpm) * 5.0 / 7.0; // scale from eRPM to RPM
    decoded.voltage = be16toh(pkt.voltage) * 0.1;
    decoded.phase_current = int16_t(be16toh(pkt.phase_current)) * 0.01;
    decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    decoded.mos_temperature = pkt.mos_temperature - 40;
    decoded.cap_temperature = pkt.cap_temperature - 40;
    decoded.status = be16toh(pkt.status);
    if (decoded.status != 0) {
        decoded.error_count++;
    }

    return true;
}

void AP_Periph_FW::xcesc_telem_update()
{
    if (!xcesc_telem.update()) {
        return;
    }
    const XCESC_Telem::XCESC& t = xcesc_telem.get_telem();

    uavcan_equipment_esc_Status pkt{};
    pkt.esc_index = g.esc_number[0]; // only supports a single ESC
    pkt.voltage = t.voltage;
    pkt.current = t.current;
    pkt.temperature = C_TO_KELVIN(MAX(t.mos_temperature, t.cap_temperature));
    pkt.rpm = t.rpm;
    pkt.power_rating_pct = t.phase_current;
    pkt.error_count = t.error_count;

    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
    canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
        UAVCAN_EQUIPMENT_ESC_STATUS_ID,
        CANARD_TRANSFER_PRIORITY_LOW,
        &buffer[0],
        total_size);
}

#endif // HAL_PERIPH_ENABLE_XCESC
