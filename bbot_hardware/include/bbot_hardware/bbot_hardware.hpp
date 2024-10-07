/*header guards to prevent multiple inclusions*/
#ifndef BBOT_HARDWARE_HPP_
#define BBOT_HARDWARE_HPP_

#include <termios.h>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "bbot_hardware/visibility_control.h"
#include "bbot_hardware/wheel.hpp"
#include "bbot_hardware/arduino_comms.hpp"


namespace bbot_hardware
{
class BbotHardware : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  float loop_rate = 0.0;
  std::string device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int enc_counts_per_rev = 0;
  int pid_p = 0;
  int pid_d = 0;
  int pid_i = 0;
  int pid_o = 0;
};

  public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BbotHardware)

  BBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  BBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  BBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  BBOT_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  BBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  BBOT_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  BBOT_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  BBOT_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  private:
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;
  Wheel wheel_r_;
  
};
}
#endif