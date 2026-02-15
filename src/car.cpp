#include "ap1/pnc_sim/car.hpp"

#include "math.h"
#include <cmath>

ap1::sim::Car::Car() {}

ap1::sim::Car::Car(
    const float x, const float y, const float z, const float yaw
): x(x), y(y), z(z), yaw(yaw) {}