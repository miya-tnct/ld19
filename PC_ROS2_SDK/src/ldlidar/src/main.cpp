#include <iostream>
#include "cmd_interface_linux.h"
#include <stdio.h>
#include "lipkg.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include "tofbf.h"
#include <string>

#define RADIAN_TO_ANGLED(angle) ((angle)*180000/3141.59)

int main(int argc , char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Node node("ldlidar");                    /* create a ROS Node */

 	
	LiPkg * lidar = new LiPkg;
  
    CmdInterfaceLinux cmd_port;
    std::vector<std::pair<std::string, std::string> > device_list;
    std::string port_name;
    cmd_port.GetCmdDevices(device_list);
    for (auto n : device_list)
    {
        std::cout << n.first << "    " << n.second << std::endl;
        if(strstr(n.second.c_str(),"CP2102"))
        {
            port_name = n.first;
        }
    }

	port_name = "/dev/ttyUSB0";

	if(port_name.empty())
	{
		std::cout<<"Can't find LiDAR LD19"<< std::endl;
	}

	std::cout<<"FOUND LiDAR_LD19"  <<std::endl;
	cmd_port.SetReadCallback([&lidar](const char *byte, size_t len) {
		if(lidar->Parse((uint8_t*)byte, len))
		{
			lidar->AssemblePacket();  
		}
	});

	if(cmd_port.Open(port_name))
		std::cout<<"LiDAR_LD19 started successfully "  <<std::endl;
	
	auto lidar_pub = node.create_publisher<LiPkg::LaserScanMsg>("scan", 1); /*create a ROS topic */
	
	while (rclcpp::ok())
	{
		if (lidar->IsFrameReady())
		{
			lidar_pub->publish(lidar->GetLaserScan());  // Fixed Frame:  lidar_frame
			lidar->ResetFrameReady();
#if 1 
			LiPkg::LaserScanMsg data = lidar->GetLaserScan();
			unsigned int lens = (data.angle_max - data.angle_min) / data.angle_increment;  
			std::cout << "current_speed: " << lidar->GetSpeed() << " " 
			          << "len: " << lens << " "
					  << "angle_min: " << RADIAN_TO_ANGLED(data.angle_min) << " "
					  << "angle_max: " << RADIAN_TO_ANGLED(data.angle_max) << std::endl; 
			std::cout << "----------------------------" << std::endl;
			for (int i = 0; i < lens; i++)
			{
				std::cout << "range: " <<  data.ranges[i] << " " 
						  << "intensites: " <<  data.intensities[i] << std::endl;
			}
			std::cout << "----------------------------" << std::endl;
#endif
		}
	}
	rclcpp::shutdown();
    return 0;
}

