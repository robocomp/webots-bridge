#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "drone_command.h"

using drone::Velocity; using drone::Limits; using drone::Source; using drone::select_target;

namespace
{
    const Limits LIM{1.0, 0.7, 1.5};
    constexpr double WD = 0.5;
    constexpr double FRESH = 0.1, STALE = 1.0;
}

TEST(SelectTarget, NothingFreshMeansHoverAtZero)
{
    const auto s = select_target(false, {}, STALE, {0.3, 0, 0, 0}, STALE, LIM, WD);
    EXPECT_EQ(s.source, Source::Hover);
    EXPECT_EQ(s.target.forward, 0.0);
    EXPECT_EQ(s.target.climb, 0.0);
}

TEST(SelectTarget, FreshProgramCommandFliesWithoutDroneMode)
{
    const auto s = select_target(false, {}, STALE, {0.3, -0.2, 0.1, 0.5}, FRESH, LIM, WD);
    EXPECT_EQ(s.source, Source::Program);
    EXPECT_DOUBLE_EQ(s.target.forward, 0.3);
    EXPECT_DOUBLE_EQ(s.target.lateral, -0.2);
    EXPECT_DOUBLE_EQ(s.target.climb, 0.1);
    EXPECT_DOUBLE_EQ(s.target.yaw, 0.5);
}

TEST(SelectTarget, ProgramCommandExpiresAfterWatchdog)
{
    const auto s = select_target(false, {}, STALE, {0.3, 0, 0, 0}, 0.51, LIM, WD);
    EXPECT_EQ(s.source, Source::Hover);
}

TEST(SelectTarget, ProgramCommandIsClampedToLimitsAndNanBecomesZero)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const auto s = select_target(false, {}, STALE, {5.0, -5.0, nan, 9.0}, FRESH, LIM, WD);
    EXPECT_DOUBLE_EQ(s.target.forward, 1.0);
    EXPECT_DOUBLE_EQ(s.target.lateral, -1.0);
    EXPECT_DOUBLE_EQ(s.target.climb, 0.0);
    EXPECT_DOUBLE_EQ(s.target.yaw, 1.5);
}

TEST(SelectTarget, OffCentreStickInDroneModeOverridesProgram)
{
    const auto s = select_target(true, {0.5, 0, 0, 0}, FRESH, {0.3, 0, 0, 0}, FRESH, LIM, WD);
    EXPECT_EQ(s.source, Source::Stick);
    EXPECT_DOUBLE_EQ(s.target.forward, 0.5 * LIM.linear);
}

TEST(SelectTarget, StickIsScaledByLimitsPerAxis)
{
    const auto s = select_target(true, {1.0, -1.0, 1.0, -1.0}, FRESH, {}, STALE, LIM, WD);
    EXPECT_DOUBLE_EQ(s.target.forward, 1.0);
    EXPECT_DOUBLE_EQ(s.target.lateral, -1.0);
    EXPECT_DOUBLE_EQ(s.target.climb, 0.7);
    EXPECT_DOUBLE_EQ(s.target.yaw, -1.5);
}

TEST(SelectTarget, CentredStickInDroneModeLetsProgramFly)
{
    const auto s = select_target(true, {0.03, 0, 0, 0}, FRESH, {0.3, 0, 0, 0}, FRESH, LIM, WD);
    EXPECT_EQ(s.source, Source::Program);
}

TEST(SelectTarget, StickIgnoredOutsideDroneMode)
{
    const auto s = select_target(false, {0.9, 0, 0, 0}, FRESH, {}, STALE, LIM, WD);
    EXPECT_EQ(s.source, Source::Hover);
}

TEST(SelectTarget, StaleStickDoesNotOverride)
{
    // A pad that stopped publishing with a stick leaning must not hold the drone forever.
    const auto s = select_target(true, {0.9, 0, 0, 0}, STALE, {0.3, 0, 0, 0}, FRESH, LIM, WD);
    EXPECT_EQ(s.source, Source::Program);
}

TEST(SelectTarget, NamesAreReadable)
{
    EXPECT_STREQ(drone::to_string(Source::Stick), "STICK");
    EXPECT_STREQ(drone::to_string(Source::Program), "PROGRAM");
    EXPECT_STREQ(drone::to_string(Source::Hover), "HOVER");
}
