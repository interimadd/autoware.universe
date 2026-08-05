// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/crosswalk_traffic_light_estimator/flashing_detection.hpp>

#include <gtest/gtest.h>

#include <utility>

using autoware::crosswalk_traffic_light_estimator::FlashingDetectionConfig;
using autoware::crosswalk_traffic_light_estimator::FlashingDetector;
using autoware::crosswalk_traffic_light_estimator::TrafficSignal;
using autoware::crosswalk_traffic_light_estimator::TrafficSignalElement;

namespace
{

const int DEFAULT_SIGNAL_ID = 100;

TrafficSignal make_signal(uint8_t color, float confidence = 1.0)
{
  TrafficSignal signal;
  signal.traffic_light_group_id = DEFAULT_SIGNAL_ID;
  TrafficSignalElement element;
  element.color = color;
  element.shape = TrafficSignalElement::CIRCLE;
  element.status = TrafficSignalElement::SOLID_ON;
  element.confidence = confidence;
  signal.elements.push_back(element);
  return signal;
}

// Helper that manages FlashingDetector
class FlashingDetectorDriver
{
public:
  explicit FlashingDetectorDriver(FlashingDetectionConfig config = FlashingDetectionConfig{1.0})
  : detector_(config)
  {
  }

  uint8_t estimate(double time_sec, uint8_t color, float confidence = 1.0)
  {
    const auto time = rclcpp::Time(static_cast<int64_t>(time_sec * 1e9));
    return detector_.estimate_stable_color(make_signal(color, confidence), time);
  }

  void clear_state(int signal_id = DEFAULT_SIGNAL_ID) { detector_.clear_state(signal_id); }

private:
  FlashingDetector detector_;
};

}  // namespace

// --- estimate_stable_color tests ---

TEST(FlashingDetectorTest, FirstCall_ReturnsDetectedColor)
{
  // Arrange
  FlashingDetectorDriver driver;

  // Act
  const uint8_t color = driver.estimate(0.0, TrafficSignalElement::GREEN);

  // Act & Assert
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, FlashingDetected_UnknownAfterGreen)
{
  // Arrange
  FlashingDetectorDriver driver;

  // Act
  driver.estimate(0.0, TrafficSignalElement::GREEN);
  const uint8_t color = driver.estimate(0.1, TrafficSignalElement::UNKNOWN);

  // Assert: maintains GREEN during flashing (UNKNOWN input doesn't change state to UNKNOWN)
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

TEST(FlashingDetectorTest, FlashingTransition_GreenToRed)
{
  // Arrange
  FlashingDetectorDriver driver;

  // Act: during flashing, RED input transitions from GREEN to RED
  driver.estimate(0.0, TrafficSignalElement::GREEN);
  driver.estimate(0.1, TrafficSignalElement::UNKNOWN);
  const uint8_t color = driver.estimate(0.2, TrafficSignalElement::RED);

  // Assert
  EXPECT_EQ(color, TrafficSignalElement::RED);
}

TEST(FlashingDetectorTest, EstimateStableColor_PrunesOldEntries)
{
  // Arrange
  FlashingDetectorDriver driver(FlashingDetectionConfig{1.0});

  // Build state and trigger flashing
  driver.estimate(0.0, TrafficSignalElement::GREEN);
  driver.estimate(0.1, TrafficSignalElement::UNKNOWN);
  driver.estimate(0.2, TrafficSignalElement::GREEN);

  // Act: feed UNKNOWN at time > hold_time, old flashing entries are pruned
  const uint8_t color = driver.estimate(2.0, TrafficSignalElement::UNKNOWN);

  // Assert: flashing was reset because old entries were pruned, only UNKNOWN remains
  EXPECT_EQ(color, TrafficSignalElement::UNKNOWN);
}

// current_time is now msg.stamp end-to-end (component test plan §3.3 (a) / §10), so it is no
// longer guaranteed to move forward between calls the way get_clock()->now() did. remove_expired_
// entries() computes a signed elapsed time and only prunes when it exceeds last_colors_hold_time;
// a negative elapsed time (stamp went backward) can never exceed a positive hold_time, so an
// older entry survives a later call with an earlier stamp. Pinned here as the accepted behavior
// (adopted option (a) of plan §3.3 (a)): this path is otherwise unreachable in the Component Test
// harness, which always merges events in ascending stamp order (plan §3.1).
TEST(FlashingDetectorTest, EstimateStableColor_ReversedStampDoesNotExpireNewerEntry)
{
  // Arrange
  FlashingDetectorDriver driver(FlashingDetectionConfig{1.0});

  // Act: a GREEN observation at t=10s, then an UNKNOWN observation at t=1s -- 9s EARLIER, well
  // past the 1.0s hold_time if elapsed were measured as |Δt|.
  driver.estimate(10.0, TrafficSignalElement::GREEN);
  const uint8_t color = driver.estimate(1.0, TrafficSignalElement::UNKNOWN);

  // Assert: the GREEN entry at t=10s was not pruned (elapsed = 1-10 = -9s, never > hold_time), so
  // both entries coexist and the detector still reads this as mid-flash -- holding GREEN, exactly
  // as it would for a forward-time flash (see FlashingDetected_UnknownAfterGreen above).
  EXPECT_EQ(color, TrafficSignalElement::GREEN);
}

// --- clear_state tests ---

TEST(FlashingDetectorTest, ClearState_RemovesTracking)
{
  // Arrange
  FlashingDetectorDriver driver;

  // Act: trigger flashing, then clear state
  driver.estimate(0.0, TrafficSignalElement::GREEN);
  driver.estimate(0.1, TrafficSignalElement::UNKNOWN);
  driver.clear_state();
  const uint8_t color = driver.estimate(0.2, TrafficSignalElement::UNKNOWN);

  // Assert: returns UNKNOWN as a first call (no flashing state)
  EXPECT_EQ(color, TrafficSignalElement::UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
