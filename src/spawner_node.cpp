#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <iostream>
#include <fstream>


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("spawner_node");

    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <sdf_path> <entity_name> [x] [y]" << std::endl;
        return 1;

    }

    std::string sdf_apth = argv[1];
    std::string entity_name = argv[2];
    float x= 0.0, y = 0.0;
    if (argc > 3) x = std::stof(argv[3]);
    if (argc > 4) y = std::stof(argv[4]);

    auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    if (!client->wait_for_service(std::chrono::seconds(10))){
        RCLCPP_ERROR(node->get_logger(),"service spawn Entity not available");
        return 1;
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = entity_name;
    request->initial_pose.position.x = x;
    request->initial_pose.position.y = y;


    std::ifstream file(sdf_apth);
    if(!file.is_open()){
        RCLCPP_ERROR(node->get_logger(),"Failed to open file %s",sdf_apth.c_str());
        rclcpp::shutdown();
        return 1;
    }

    std::stringstream sdf_stream;
    sdf_stream << file.rdbuf();
    request->xml = sdf_stream.str();


    auto future = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node,future) != rclcpp::FutureReturnCode::SUCCESS){
        auto response = future.get();
        if(response){
            RCLCPP_INFO(node->get_logger(),"Spawned entity successfully");
        }
        else{
            RCLCPP_ERROR(node->get_logger(),"Failed to spawn entity");

        }
    }else{
        RCLCPP_ERROR(node->get_logger(),"Service call failed");

    }

        // Shutdown ROS 2
    RCLCPP_INFO(node->get_logger(), "Done! Shutting down node.");
    rclcpp::shutdown();
    return 0;
}
