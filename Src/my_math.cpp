#include "my_math.h"

#include <math.h>

namespace my_math
{
    const float faraday_constant = 96485; // C/mol
    const float universal_gas_constant = 8.314;
    const float atmospheric_oxygen_partial_pressure = 21331;

    float volts_to_volume_concentration(float v, float temperature)
    {
        float log_pressure_ratio = -v * (4 * faraday_constant / universal_gas_constant) / temperature;
        return exp(log_pressure_ratio) * atmospheric_oxygen_partial_pressure;
    }
} // namespace my_math
