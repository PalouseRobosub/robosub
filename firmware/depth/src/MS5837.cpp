#include "MS5837.h"
#include <Wire.h>
#include <ros.h>

MS5837::MS5837()
{
    fluidDensity = 1029;
    converting = false;
}

int MS5837::init()
{
    // Reset the MS5837, per datasheet
    Wire.beginTransmission(ms5837_address);
    if (Wire.write(static_cast<uint8_t>(Command::Reset)) != 1)
    {
        return -1;
    }

    if (Wire.endTransmission())
    {
        return -1;
    }

    // Wait for reset to complete
    delay(10);

    // Read calibration values and CRC
    for (uint8_t i = 0 ; i < 7 ; i++)
    {
        Wire.beginTransmission(ms5837_address);
        if (Wire.write(static_cast<uint8_t>(Command::PromRead) + i * 2) != 1)
        {
            return -1;
        }

        if (Wire.endTransmission())
        {
            return -1;
        }

        if (Wire.requestFrom(ms5837_address, 2) != 2)
        {
            return -1;
        }

        C[i] = (Wire.read() << 8) | Wire.read();
    }

    // Verify that data is correct with CRC
    uint8_t crcRead = C[0] >> 12;

    return (crc4(C) != crcRead);
}

void MS5837::setFluidDensity(float density)
{
    fluidDensity = density;
}

int MS5837::trigger_read(Measurement sensor, ros::Time now)
{
    const uint8_t data = ((sensor == Measurement::Pressure)?
        static_cast<uint8_t>(Command::ConvertD1_8192) :
        static_cast<uint8_t>(Command::ConvertD2_8192));

    /*
     * Trigger the specified conversion to begin.
     */
    Wire.beginTransmission(ms5837_address);
    if (Wire.write(data) != 1)
    {
        return -1;
    }

    if (Wire.endTransmission())
    {
        return -1;
    }

    /*
     * The conversion will be ready after 20ms.
     */
    conversion = sensor;
    converting = true;
    conversion_ready = now;
    conversion_ready += ros::Duration(0, 20000000);

    return 0;
}

int MS5837::read(ros::Time now)
{
    /*
     * If the conversion hasn't completed yet, return immediately.
     */
    if (now.toSec() < conversion_ready.toSec())
    {
        return 0;
    }

    /*
     * If the user requests a read without queuing a conversion, return an
     * error.
     */
    if (converting == false)
    {
        return -1;
    }

    /*
     * Request the ADC result and read the conversion.
     */
    Wire.beginTransmission(ms5837_address);
    if (Wire.write(static_cast<uint8_t>(Command::AdcRead)) != 1)
    {
        return -1;
    }

    if (Wire.endTransmission())
    {
        return -1;
    }

    if (Wire.requestFrom(ms5837_address, 3) != 3)
    {
        return -1;
    }

    /*
     * Store the conversion in either the pressure or temperature measurement.
     */
    if (conversion == Measurement::Pressure)
    {
        D1 = 0;
        D1 = Wire.read();
        D1 = (D1 << 8) | Wire.read();
        D1 = (D1 << 8) | Wire.read();
    }
    else
    {
        D2 = 0;
        D2 = Wire.read();
        D2 = (D2 << 8) | Wire.read();
        D2 = (D2 << 8) | Wire.read();
    }

    calculate();
    converting = false;

    /*
     * Return a one to indicate the read completed successfully.
     */
    return 1;
}

void MS5837::readTestCase()
{
    C[0] = 0;
    C[1] = 34982;
    C[2] = 36352;
    C[3] = 20328;
    C[4] = 22354;
    C[5] = 26646;
    C[6] = 26146;
    C[7] = 0;

    D1 = 4958179;
    D2 = 6815414;

    calculate();
}

void MS5837::calculate()
{
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation
    int32_t dT;
    int64_t SENS;
    int64_t OFF;
    int32_t SENSi;
    int32_t OFFi;
    int32_t Ti;
    int64_t OFF2;
    int64_t SENS2;

    // Terms called
    dT = D2 - uint32_t(C[5])*256l;
    SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
    OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;

    // Temp and P conversion
    TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;
    P = (D1 * SENS / (2097152l) - OFF) / (8192l);

    // Second order compensation
    if ((TEMP / 100) < 20)
    {
        // Low temperature
        Ti = (3 * int64_t(dT) * int64_t(dT)) / (8589934592LL);
        OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
        SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
        if ((TEMP / 100) < -15)
        {
            // Very low temp
            OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
            SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
        }
    }
    else if ((TEMP / 100) >= 20)
    {
        // High temperature
        Ti = 2 * (dT * dT) / (137438953472LL);
        OFFi = ((TEMP - 2000) * (TEMP - 2000)) / 16;
        SENSi = 0;
    }

    // Calculate pressure and temp second order
    OFF2 = OFF - OFFi;
    SENS2 = SENS - SENSi;

    TEMP = (TEMP - Ti);
    P = (((D1 * SENS2) / 2097152l - OFF2) / 8192l);
}

float MS5837::pressure(float conversion)
{
    return P / 10.0f * conversion;
}

float MS5837::temperature()
{
    return TEMP / 100.0f;
}

float MS5837::depth()
{
    return (pressure(MS5837::Pa) - 101300) / (fluidDensity * 9.80665);
}

float MS5837::altitude()
{
    return (1 - pow((pressure() / 1013.25), .190284)) * 145366.45 * .3048;
}


uint8_t MS5837::crc4(uint16_t n_prom[])
{
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    for (uint8_t i = 0 ; i < 16; i++)
    {
        if (i % 2 == 1)
        {
            n_rem ^= (uint16_t)((n_prom[i>>1]) & 0x00FF);
        }
        else
        {
            n_rem ^= (uint16_t)(n_prom[i>>1] >> 8);
        }

        for (uint8_t n_bit = 8 ; n_bit > 0 ; n_bit--)
        {
            if (n_rem & 0x8000)
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }

    n_rem = ((n_rem >> 12) & 0x000F);

    return n_rem ^ 0x00;
}
