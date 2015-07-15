#pragma once

#include <cstdint>
#include <limits>

#include "bit_field.h"

namespace nihstro {

inline uint32_t to_float24(float val) {
    static_assert(std::numeric_limits<float>::is_iec559, "Compiler does not adhere to IEEE 754");

    union Float32 {
        BitField< 0, 23, uint32_t> mant;
        BitField<23,  8, uint32_t> biased_exp;
        BitField<31,  1, uint32_t> sign;

        static int ExponentBias() {
            return 127;
        }
    } f32 = reinterpret_cast<Float32&>(val);

    union Float24 {
        uint32_t hex;

        BitField< 0, 16, uint32_t> mant;
        BitField<16,  7, uint32_t> biased_exp;
        BitField<23,  1, uint32_t> sign;

        static int ExponentBias() {
            return 63;
        }
    } f24 = { 0 };

    int biased_exp = (int)f32.biased_exp - Float32::ExponentBias() + Float24::ExponentBias();
    unsigned mant = (biased_exp >= 0) ? (f32.mant >> (f32.mant.NumBits() - f24.mant.NumBits())) : 0;
    if (biased_exp >= (1 << f24.biased_exp.NumBits())) {
        // TODO: Return +inf or -inf
    }

    f24.biased_exp = std::max(0, biased_exp);
    f24.mant = mant;
    f24.sign = f32.sign.Value();

    return f24.hex;
}

} // namespace
