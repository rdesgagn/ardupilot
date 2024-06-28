/*
  generic object to allow a script to use a serial driver stream from both
  driver and device perspectives
 */

#include "AP_Scripting_config.h"
#include "AP_Scripting.h"
#include "AP_Scripting_SerialAccess.h"

#if AP_SCRIPTING_ENABLED

#if AP_SCRIPTING_SERIALDEVICE_ENABLED
#define check_is_device_port() (is_device_port)
#define ON_DEVICE_PORT(func, ...) (((AP_Scripting_SerialDevice::Port*)stream)->device_##func (__VA_ARGS__))
#else
#define check_is_device_port() (false)
#define ON_DEVICE_PORT(...) (0) // not executed
#endif

void AP_Scripting_SerialAccess::begin(uint32_t baud)
{
    if (!check_is_device_port()) {
        stream->begin(baud);
    }
}

int32_t AP_Scripting_SerialAccess::write(uint8_t c)
{
    return (int32_t)write(&c, 1); // return value will be 0 or 1
}

size_t AP_Scripting_SerialAccess::write(const uint8_t *buffer, size_t size)
{
    if (!check_is_device_port()) {
        return stream->write(buffer, size);
    }
    return ON_DEVICE_PORT(write, buffer, size);
}

bool AP_Scripting_SerialAccess::read(uint8_t &c)
{
    return read(&c, 1) > 0;
}

ssize_t AP_Scripting_SerialAccess::read(uint8_t* buffer, uint16_t count)
{
    if (!check_is_device_port()) {
        return stream->read(buffer, count);
    }
    return ON_DEVICE_PORT(read, buffer, count);
}

int32_t AP_Scripting_SerialAccess::available(void)
{
    uint32_t avail;
    if (!check_is_device_port()) {
        avail = stream->available();
    } else {
        avail = ON_DEVICE_PORT(available);
    }
    return MIN(avail, (uint32_t)INT32_MAX); // ensure result fits in a Lua integer
}

void AP_Scripting_SerialAccess::set_flow_control(enum AP_HAL::UARTDriver::flow_control fcs)
{
    if (!check_is_device_port()) {
        stream->set_flow_control(fcs);
    }
}

#endif // AP_SCRIPTING_ENABLED
