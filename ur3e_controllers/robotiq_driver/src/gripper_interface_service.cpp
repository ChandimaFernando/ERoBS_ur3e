// Copyright (c) 2022 PickNik, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "custom_msgs/srv/gripper_cmd.hpp"

#include <robotiq_driver/robotiq_gripper_interface.hpp>

constexpr auto kComPort = "/tmp/ttyUR";
constexpr auto kSlaveID = 0x09;

RobotiqGripperInterface gripper(kComPort, kSlaveID);

void gripper_controller(const std::shared_ptr<custom_msgs::srv::GripperCmd::Request> request, std::shared_ptr<custom_msgs::srv::GripperCmd::Response> response){

  // This function sends the gripper control command
  bool status = false ;
  try
  {
    char cmd = request->cmd ;

    switch (cmd)
    {
    case 'A':
      // Activate the gripper
      gripper.deactivateGripper();
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      gripper.activateGripper();
      RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful"); 

      break;

    case 'D':
      // Deactivate the gripper
      gripper.deactivateGripper();
      RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Deactivated"); 

      break;

    case 'M':
      {
        // Closes the gripper to the percentage set by request->grip
        uint8_t val = request->grip*2.55 ; // convert the scales from 01-100 to 0-255        
        // std::cout << "######### request and val : " << std::endl ; 
        // std::cout << static_cast<int16_t>(request->grip) << std::endl ; 
        // std::cout << static_cast<int16_t>(val) << std::endl ; 
        gripper.setGripperPosition(val);
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open at %d ", val);   
      }
      break;

    case 'O':
      /* Open the grippper fully */
      gripper.setGripperPosition(0x00);
      RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");   
      break;

    case 'C':
      /* Close the grippper fully */
      gripper.setGripperPosition(0xFF);
      RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Close");   
      break;

    // case 'S':
    //   /* Close the grippper fully */
    //   gripper.setGripperPosition(0xFF);
    //   break;

    default:
      break;
    }

    status = true ;
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    status = false ;
  }

  // Send the response back
  response->status = status;

}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gripper_service");   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activate the gripper ..."); 
  // Clear the registers
  gripper.deactivateGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Activate the gripper
  gripper.activateGripper();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  gripper.setSpeed(0x0F);

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful"); 

  rclcpp::Service<custom_msgs::srv::GripperCmd>::SharedPtr service = node->create_service<custom_msgs::srv::GripperCmd>("gripper_service",  &gripper_controller);   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Ready to recieve gripper commands.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
  
}
