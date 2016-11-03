#include "utility/math.hpp"


namespace rs
{
    namespace math
    {
        int sign(double x)
        {
            return (x > 0) - (x < 0);
        }
    }
};
