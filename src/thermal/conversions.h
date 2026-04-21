#pragma once

namespace ungula {
    namespace thermal {

        inline double celsiusToFahrenheit(double c) {
            return c * 9.0 / 5.0 + 32.0;
        }

        inline double fahrenheitToCelsius(double f) {
            return (f - 32.0) * 5.0 / 9.0;
        }

    }  // namespace thermal
}  // namespace ungula