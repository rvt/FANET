#pragma once

#include <stdint.h>
#include <etl/math.h>
#include <etl/power.h>
#include <etl/ratio.h>
#include <etl/algorithm.h>

namespace FANET
{
    /**
     * @brief Scales a floating-point number to a fixed-point representation.
     *
     * This function scales a floating-point number to a fixed-point representation
     * based on the provided unit factor, scaling factor, and bit count. It ensures
     * that the scaled value fits within the specified bit count and clamps the result
     * if necessary.
     *
     * @tparam R The return type (fixed-point representation).
     * @tparam UnitFactor The unit factor for scaling.
     * @tparam ScalingFactor The scaling factor for scaling.
     * @tparam bitCount The number of bits for the fixed-point representation.
     * @param number The floating-point number to scale.
     * @return A struct containing the scaled value and a boolean indicating if scaling was applied.
     */
    template <typename R, typename UnitFactor, typename ScalingFactor, uint8_t bitCount>
    auto toScaled(float number)
    {
        struct Result
        {
            R value;     // The scaled value.
            bool scaled; // Indicates if scaling was applied.
        };

        // Compute constants
        constexpr float unitFactor = static_cast<float>(UnitFactor::num) / UnitFactor::den;
        constexpr float scalingFactor = static_cast<float>(ScalingFactor::num) / ScalingFactor::den;
        constexpr uint8_t maxBits = bitCount - static_cast<uint8_t>(std::is_signed<R>::value);
        constexpr int32_t constrainedMax = (1 << maxBits) - 1;

        // Ensure non-negative values for unsigned types
        if constexpr (etl::is_unsigned<R>::value)
        {
            number = etl::max(0.0f, number);
        }

        float ret = roundf(number / unitFactor);

        // Check if the number fits without scaling
        if (etl::absolute(ret) <= constrainedMax)
        {
            return Result{
                etl::clamp(static_cast<R>(ret), static_cast<R>(etl::is_unsigned<R>::value ? 0 : -constrainedMax), static_cast<R>(constrainedMax)),
                false};
        }

        // Scale and clamp the result
        float scaled = roundf(number / scalingFactor);

        return Result{
            etl::clamp(static_cast<R>(scaled), static_cast<R>(etl::is_unsigned<R>::value ? 0 : -constrainedMax), static_cast<R>(constrainedMax)),
            true};
    }

    // /**
    //  * @brief Calculate the airtime of a LoRa packet.
    //  *
    //  * This function calculates the airtime of a LoRa packet based on the provided parameters.
    //  *
    //  * @param size The size of the payload in bytes.
    //  * @param sf The spreading factor.
    //  * @param bw The bandwidth in kHz.
    //  * @param cr The coding rate (1:4/5, 2:4/6, 3:4/7, 4:4/8).
    //  * @param lowDrOptimize Low Data Rate Optimization (2:auto, 1:yes, 0:no).
    //  * @param explicitHeader True for explicit header, false for implicit header.
    //  * @param preambleLength The length of the preamble.
    //  * @return The airtime in milliseconds.
    //  */
    // float LoraAirtimeFloat(int size,
    //                             int sf,
    //                             int bw,
    //                             int cr = 1, // Coding Rate in 1:4/5 2:4/6 3:4/7 4:4/8
    //                             int lowDrOptimize = 2 /*2:auto 1:yes 0:no*/,
    //                             bool explicitHeader = true,
    //                             int preambleLength = 8)
    // {
    //     // Symbol time in milliseconds
    //     float tSym = (std::pow(2, sf) / (bw * 1000)) * 1000.0;

    //     // Preamble time
    //     float tPreamble = (preambleLength + 4.25) * tSym;

    //     // Header: 0 when explicit, 1 when implicit
    //     int h = explicitHeader ? 0 : 1;

    //     // Low Data Rate Optimization (DE)
    //     int de = ((lowDrOptimize == 2 && bw == 125 && sf >= 11) || lowDrOptimize == 1) ? 1 : 0;

    //     // Calculate number of payload symbols
    //     int payloadSymbNb = 8 + etl::max(
    //                                 (int)std::ceil((8 * size - 4 * sf + 28 + 16 - 20 * h) / (4.0 * (sf - 2 * de))) * (cr + 4),
    //                                 0);

    //     // Payload time
    //     float tPayload = payloadSymbNb * tSym;

    //     // Total airtime in milliseconds
    //     return tPreamble + tPayload;
    // }

    /**
     * @brief Calculate the airtime of a LoRa packet (integer version).
     *
     * This function calculates the airtime of a LoRa packet based on the provided parameters.
     * It uses integer arithmetic for the calculations.
     *
     * @param size The size of the payload in bytes.
     * @param sf The spreading factor.
     * @param bw The bandwidth in kHz.
     * @param cr The coding rate (1:4/5, 2:4/6, 3:4/7, 4:4/8).
     * @param lowDrOptimize Low Data Rate Optimization (2:auto, 1:yes, 0:no).
     * @param explicitHeader True for explicit header, false for implicit header.
     * @param preambleLength The length of the preamble.
     * @return The airtime in milliseconds.
     */
    static int32_t LoraAirtime(int size,
                               int sf,
                               int bw,
                               int cr = 1, // Coding Rate in 1:4/5 2:4/6 3:4/7 4:4/8
                               int lowDrOptimize = 2 /*2:auto 1:yes 0:no*/,
                               bool explicitHeader = true,
                               int preambleLength = 8)
    {
        // Symbol time in milliseconds
        int32_t tSym = 1 << sf;

        // Preamble time
        int32_t tPreamble = ((preambleLength * 4 + (4 * 4.25)) * tSym) / bw / 4;

        // Header: 0 when explicit, 1 when implicit
        int32_t h = explicitHeader ? 0 : 1;

        // Low Data Rate Optimization (DE)
        int32_t de = ((lowDrOptimize == 2 && bw == 125 && sf >= 11) || lowDrOptimize == 1) ? 1 : 0;

        // Calculate number of payload symbols
        int32_t payloadSymbNb = 8 + etl::max(
                                        (int)std::ceil((8 * size - 4 * sf + 28 + 16 - 20 * h) / (4.0 * (sf - 2 * de))) * (cr + 4),
                                        0);

        // Payload time
        int32_t tPayload = (payloadSymbNb * tSym) / bw;

        // Total airtime in milliseconds
        return tPreamble + tPayload;
    }

    /**
     * @brief calculate the time on air using an EMA filter
     * This will never be a true average and always be an approximate
     * Used within the protocol class to decide if frames can be send or not
     */
    class Airtime
    {

    public:
        Airtime(uint32_t windowMs = 30000)
            : totalAirtime(0), lastCleanupTime(0), windowMs(windowMs) {}

        void set(uint32_t currentTimeMs, uint16_t timeOnAirMs)
        {
            cleanup(currentTimeMs);
            totalAirtime += timeOnAirMs;
        }

        uint32_t get(uint32_t currentTimeMs)
        {
            cleanup(currentTimeMs);
            return (totalAirtime * 1000) / windowMs; // Integer math, scaled by 1000
        }

        uint32_t getAverage() const
        {
            return (totalAirtime * 1000) / windowMs; // Integer math, scaled by 1000
        }

    private:
        uint32_t totalAirtime;
        uint32_t lastCleanupTime;
        const uint32_t windowMs;

        void cleanup(uint32_t currentTimeMs)
        {
            if (lastCleanupTime == 0)
            {
                lastCleanupTime = currentTimeMs;
                return;
            }

            uint32_t elapsed = currentTimeMs - lastCleanupTime;
            if (elapsed >= windowMs)
            {
                totalAirtime = 0;
            }
            else
            {
                uint32_t decayAmount = (totalAirtime * elapsed) / windowMs;
                totalAirtime = (decayAmount > totalAirtime) ? 0 : (totalAirtime - decayAmount);
            }

            lastCleanupTime = currentTimeMs;
        }
    };
}
