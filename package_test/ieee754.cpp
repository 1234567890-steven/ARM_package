#include "Arduino.h"
#include "ieee754.h"

union FloatUnion {
    float f;
    uint32_t u;
};

IEEE754::IEEE754 () {}

void IEEE754::float_to_hex(float num, byte (&data)[4])
{
    FloatUnion fu;
    fu.f = num;
    uint32_t binary_value = fu.u;
    data[0] = (binary_value >> 24) & 0xFF;
    data[1] = (binary_value >> 16) & 0xFF;
    data[2] = (binary_value >> 8) & 0xFF;
    data[3] = binary_value & 0xFF;
}

float IEEE754::hex_to_float(byte data[4])
{
    FloatUnion fu;
    uint32_t binary_value = 0;
    binary_value |= (data[0] << 24);
    binary_value |= (data[1] << 16);
    binary_value |= (data[2] << 8);
    binary_value |= data[3];
    fu.u = binary_value;
    return fu.f;
}