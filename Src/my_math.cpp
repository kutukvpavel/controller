#include "my_math.h"

#include <math.h>
#include "user.h"

namespace my_math
{
    const float faraday_constant = 96485; // C/mol
    const float universal_gas_constant = 8.314;
    const float atmospheric_oxygen_partial_pressure = 21331;

    float volts_to_partial_pressure(float v, float temperature)
    {
        assert_param(temperature > 0);
        assert_param(isfinite(v));

        float log_pressure_ratio = v * (4 * faraday_constant / universal_gas_constant) / temperature;
        return exp(log_pressure_ratio) * atmospheric_oxygen_partial_pressure;
    }
} // namespace my_math
