// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <string>
#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "falcon/core/FalconDevice.h" 
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconCLIBase.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
#include "falcon/core/FalconGeometry.h"
#include "falcon/gmtl/gmtl.h"


#include "falcon_interfaces/msg/falcon_pos.hpp"
#include "falcon_interfaces/msg/falcon_forces.hpp"
using namespace std;
using namespace libnifalcon;
using namespace StamperKinematicImpl;

using namespace std::chrono_literals;

std::shared_ptr<FalconFirmware> f;
FalconKinematic* k;
unsigned int num_falcons = 0;
int cc = 0;
FalconDevice dev;

class ReadFalcon : public rclcpp::Node
{
  public:
    std::shared_ptr<FalconFirmware> f;
	  FalconKinematic* k;
	  unsigned int num_falcons = 0;
	  int cc = 0;
	  FalconDevice dev;
    std::array<int, 3> pos;
    bool state = false;

    ReadFalcon()
    : Node("read_falcon"), count_(0)
    {
      //FalconDevice dev;
      publisher_ = this->create_publisher<falcon_interfaces::msg::FalconPos>("readFalcon", 10);
      // if(runFalconTest() != true){
      //   runFalconTest();
      // }
      // else{
      //   state = true;
      // }
      dev.setFalconFirmware<FalconFirmwareNovintSDK>();
      f = dev.getFalconFirmware();
      if(!dev.open(num_falcons))
		  {
			  std::cout << "Cannot open falcon - Error: " << dev.getErrorCode() << std::endl;
		  }

		  if(!dev.isFirmwareLoaded())
		  {
			  std::cout << "Loading firmware" << std::endl;
				  if(!dev.getFalconFirmware()->loadFirmware(false, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
				  {
					  std::cout << "Could not load firmware" << std::endl;
				  }
					std::cout <<"Firmware loaded" << std::endl;
            f->setLEDStatus(0);
            timer_ = this->create_wall_timer(
            100ms, std::bind(&ReadFalcon::timer_callback, this));
	    }
      else{
        timer_ = this->create_wall_timer(
        100ms, std::bind(&ReadFalcon::timer_callback, this));
      }	
      // dev.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
      // timer_ = this->create_wall_timer(
      // 100ms, std::bind(&ReadFalcon::timer_callback, this));
    }

    // void runFalconTest()
    // {
    //   dev.setFalconFirmware<FalconFirmwareNovintSDK>();
    //   f = dev.getFalconFirmware();
    //   if(state != true){
    //     if(!dev.open(num_falcons))
		//   {
		// 	  std::cout << "Cannot open falcon - Error: " << dev.getErrorCode() << std::endl;
		//   }

		//   if(!dev.isFirmwareLoaded())
		//   {
		// 	  std::cout << "Loading firmware" << std::endl;
		// 		  if(!dev.getFalconFirmware()->loadFirmware(false, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
		// 		  {
		// 			  std::cout << "Could not load firmware" << std::endl;
		// 		  }
		// 			  std::cout <<"Firmware loaded" << std::endl;
    //         f->setLEDStatus(0);
    //         state = true;
	  //   }
    //   else{
    //     state = true;
    //   }	
		  
    // } 

    void timer_callback()
    {
      dev.setFalconKinematic<libnifalcon::FalconKinematicStamper>();
      //std::cout <<"sending Enc" << std::endl;
      std::array<double, 3> pos;
      dev.runIOLoop();
      pos = dev.getPosition();
      //printf("count: %5d",cc++);
      //printf("| Enc1: %5d | Enc2: %5d | Enc3: %5d \n",f->getEncoderValues()[0], f->getEncoderValues()[1], f->getEncoderValues()[2]);
      //printf("| p1: %5d | p2: %5d | p3: %5d \n",pos[0], pos[1], pos[2]);
      auto message = falcon_interfaces::msg::FalconPos();
      message.x = pos[0];
      message.y = pos[1];
      message.z = pos[2];
      publisher_->publish(message);
    }

    

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<falcon_interfaces::msg::FalconPos>::SharedPtr publisher_;
    size_t count_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReadFalcon>());
  rclcpp::shutdown();
  return 0;
}