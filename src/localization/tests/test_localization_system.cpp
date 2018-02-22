/* 
file: test_localization_system.cpp
A file that uses the gtest suite inside of ros to run unit tests
and integration tests.
*/

//System includes
#include <gtest/gtest.h>
#include <vector>
#include <string>

//Project includes
#include "ros/ros.h"
#include "../particle_filter.h"
#include "../lin_accel_kalman_filter.h"
#include "../robosub_sensors.h"

TEST(ParticleFilter, HandlesGettersAndSetters)
{
    EXPECT_TRUE(true) << "This trivial test should never fail.";
}
