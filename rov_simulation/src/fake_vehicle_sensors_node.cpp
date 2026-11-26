// Copyright 2026 sam
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

// fake_vehicle_sensors_node: simulated battery and leak sensors for the
// safety monitor (implementation step 10).
//
// Publishes at publish_rate_hz:
//   battery_state  sensor_msgs/BatteryState, percentage = battery_percentage
//                  (0..1), minus discharge_per_s * elapsed time;
//   leak_alarm     rov_interfaces/LeakAlarm, leak = leak.
// battery_percentage, leak and stamp_offset_s (added to the header stamps:
// negative = stale readings) can be set at run time (scripted tests).

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rov_interfaces/msg/leak_alarm.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rov_simulation
{

class FakeVehicleSensorsNode : public rclcpp::Node
{
public:
  FakeVehicleSensorsNode()
  : Node("fake_vehicle_sensors_node")
  {
    const double rate_hz = declare_parameter("publish_rate_hz", 1.0);
    percentage_ = declare_parameter("battery_percentage", 0.9);
    discharge_ = declare_parameter("discharge_per_s", 0.0);
    leak_ = declare_parameter("leak", false);
    stamp_offset_ = declare_parameter("stamp_offset_s", 0.0);
    if (!(rate_hz > 0.0) || percentage_ < 0.0 || percentage_ > 1.0 || discharge_ < 0.0) {
      throw std::invalid_argument(
              "publish_rate_hz > 0, 0 <= battery_percentage <= 1, discharge_per_s >= 0");
    }
    since_ = now();
    callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const rclcpp::Parameter & parameter : parameters) {
          if (parameter.get_name() == "battery_percentage") {
            percentage_ = parameter.as_double();
            since_ = now();
            RCLCPP_WARN(get_logger(), "Battery set to %.0f %%", 100.0 * percentage_);
          } else if (parameter.get_name() == "stamp_offset_s") {
            stamp_offset_ = parameter.as_double();
          } else if (parameter.get_name() == "leak") {
            leak_ = parameter.as_bool();
            RCLCPP_WARN(get_logger(), "Leak sensor %s", leak_ ? "WET" : "dry");
          }
        }
        return result;
      });
    battery_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    leak_publisher_ = create_publisher<rov_interfaces::msg::LeakAlarm>("leak_alarm", 10);
    timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::duration<double>(1.0 / rate_hz), [this]() {Publish();});
  }

private:
  void Publish()
  {
    const rclcpp::Time stamp = now();
    const rclcpp::Time stamped = stamp + rclcpp::Duration::from_seconds(stamp_offset_);
    sensor_msgs::msg::BatteryState battery;
    battery.header.stamp = stamped;
    battery.percentage = static_cast<float>(
      std::max(0.0, percentage_ - discharge_ * (stamp - since_).seconds()));
    battery.present = true;
    battery_publisher_->publish(battery);
    rov_interfaces::msg::LeakAlarm leak;
    leak.header.stamp = stamped;
    leak.leak = leak_;
    leak_publisher_->publish(leak);
  }

  double percentage_;
  double discharge_;
  double stamp_offset_ = 0.0;
  bool leak_;
  rclcpp::Time since_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_publisher_;
  rclcpp::Publisher<rov_interfaces::msg::LeakAlarm>::SharedPtr leak_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
};

}  // namespace rov_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_simulation::FakeVehicleSensorsNode>());
  rclcpp::shutdown();
  return 0;
}
