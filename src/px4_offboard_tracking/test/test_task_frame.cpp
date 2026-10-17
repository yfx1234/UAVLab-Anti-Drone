#include <gtest/gtest.h>

#include "px4_offboard_tracking/task_frame.hpp"

using px4_offboard_tracking::NedVector3;
using px4_offboard_tracking::compensate_px4_local_reset;
using px4_offboard_tracking::height_above_task_origin_m;
using px4_offboard_tracking::to_task_ned;

TEST(TaskFrame, PropellerStartIsExactOrigin)
{
  const NedVector3 propeller_start{12.0, -4.0, 1.5};
  const auto task = to_task_ned(propeller_start, propeller_start);
  EXPECT_DOUBLE_EQ(task[0], 0.0);
  EXPECT_DOUBLE_EQ(task[1], 0.0);
  EXPECT_DOUBLE_EQ(task[2], 0.0);
}

TEST(TaskFrame, HeightAndTargetCoordinatesShareTheSameOrigin)
{
  const NedVector3 propeller_start{10.0, 20.0, -2.0};
  const NedVector3 pursuer{13.0, 16.0, -7.0};
  const auto pursuer_task = to_task_ned(pursuer, propeller_start);
  EXPECT_DOUBLE_EQ(pursuer_task[0], 3.0);
  EXPECT_DOUBLE_EQ(pursuer_task[1], -4.0);
  EXPECT_DOUBLE_EQ(pursuer_task[2], -5.0);
  EXPECT_DOUBLE_EQ(height_above_task_origin_m(pursuer, propeller_start), 5.0);

  const NedVector3 target_task{
    pursuer_task[0], pursuer_task[1], pursuer_task[2] - 25.0};
  EXPECT_DOUBLE_EQ(target_task[2], -30.0);
  EXPECT_DOUBLE_EQ(pursuer_task[2] - target_task[2], 25.0);
}

TEST(TaskFrame, Px4ResetDoesNotMoveThePhysicalTaskCoordinate)
{
  NedVector3 origin{10.0, 20.0, -2.0};
  NedVector3 position{13.0, 16.0, -7.0};
  const NedVector3 delta{100.0, -50.0, 8.0};
  const auto before = to_task_ned(position, origin);
  compensate_px4_local_reset(origin, delta);
  compensate_px4_local_reset(position, delta);
  const auto after = to_task_ned(position, origin);
  EXPECT_EQ(after, before);
}
