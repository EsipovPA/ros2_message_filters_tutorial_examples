
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "message_filters/subscriber.hpp"
#include "message_filters/synchronizer.hpp"
#include "message_filters/sync_policies/approximate_epsilon_time.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class EpsTimeSyncNode : public rclcpp::Node {
public:
  typedef sensor_msgs::msg::Temperature TemperatureMsg;
  typedef sensor_msgs::msg::FluidPressure FluidPressureMsg;
  typedef message_filters::sync_policies::ApproximateEpsilonTime<
    sensor_msgs::msg::Temperature,
    sensor_msgs::msg::FluidPressure
  > SyncPolicy;

  EpsTimeSyncNode(): Node("epsilon_time_sync_node") {
    rclcpp::QoS qos = rclcpp::QoS(10);
    temp_pub = this->create_publisher<TemperatureMsg>("temp", qos);
    fluid_pub = this->create_publisher<FluidPressureMsg>("fluid", qos);

    temp_sub.subscribe(this, "temp", qos);
    fluid_sub.subscribe(this, "fluid", qos);

    temperature_timer = this->create_wall_timer(500ms, std::bind(&EpsTimeSyncNode::TemperatureTimerCallback, this));
    fluid_pressure_timer = this->create_wall_timer(550ms, std::bind(&EpsTimeSyncNode::FluidPressureTimerCallback, this));

    uint32_t queue_size = 10;
    sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>> (
      SyncPolicy (
        queue_size,
        // rclcpp::Duration(std::chrono::seconds(1))
        rclcpp::Duration(std::chrono::nanoseconds(5000000))
      ),
      temp_sub,
      fluid_sub
    );

    sync->registerCallback(std::bind(&EpsTimeSyncNode::SyncCallback, this, _1, _2));
  }

private:

  void SyncCallback(
    const TemperatureMsg::ConstSharedPtr & temp,
    const FluidPressureMsg::ConstSharedPtr & fluid
  ) {
    RCLCPP_INFO(
      this->get_logger(),
      "Sync callback with %u:%u and %u:%u as times",
      temp->header.stamp.sec,
      temp->header.stamp.nanosec,
      fluid->header.stamp.sec,
      fluid->header.stamp.nanosec
    );

    if (temp->temperature > 2.0)
    {
      FluidPressureMsg new_fluid;
      new_fluid.header.stamp = rclcpp::Clock().now();
      new_fluid.header.frame_id = "test";
      new_fluid.fluid_pressure = 2.5;
      fluid_pub->publish(new_fluid);
    }
  }

  void TemperatureTimerCallback()
  {
    TemperatureMsg temp;
    auto now = this->get_clock()->now();
    temp.header.stamp = now;
    temp.header.frame_id = "test";
    temp.temperature = 1.0;
    temp_pub->publish(temp);
  }

  void FluidPressureTimerCallback()
  {
    FluidPressureMsg fluid;
    auto now = this->get_clock()->now();
    fluid.header.stamp = now;
    fluid.header.frame_id = "test";
    fluid.fluid_pressure = 2.0;
    fluid_pub->publish(fluid);
  }

private:
  rclcpp::Publisher<TemperatureMsg>::SharedPtr temp_pub;
  rclcpp::Publisher<FluidPressureMsg>::SharedPtr fluid_pub;

  message_filters::Subscriber<TemperatureMsg> temp_sub;
  message_filters::Subscriber<FluidPressureMsg> fluid_sub;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

  rclcpp::TimerBase::SharedPtr temperature_timer;
  rclcpp::TimerBase::SharedPtr fluid_pressure_timer;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EpsTimeSyncNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}