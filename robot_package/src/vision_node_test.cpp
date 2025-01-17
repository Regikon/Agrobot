/**
**  Simple ROS Node
**/
#include <ros/ros.h>
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>             // for cout
#include <algorithm>            // min and max
#include <chrono>               // time

using namespace rs2;



int main(int argc, char * argv[]) try
{
    ros::init(argc, argv, "vision_node");
    // Create a ROS node handle
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    ///////////////////////////////////////////////////////////////////////////
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;

    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();



    ///////////////////////////////////////////////////////////////////////////
    while (ros::ok())
    {
        auto start = std::chrono::steady_clock::now();
        // Block program until frames arrive
        rs2::frameset frames = pipe.wait_for_frames();

        // Try to get a frame of a depth image and color image
        rs2::depth_frame depth = frames.get_depth_frame();
        rs2::video_frame color = frames.get_color_frame();


        // Tell pointcloud object to map to this color frame
        pc.map_to(color);

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);

        auto vertices = points.get_vertices();              // get vertices
        auto tex_coords = points.get_texture_coordinates(); // and texture coordinates


        // for (int i = 0; i < points.size(); i++)
        // {
        //     if (vertices[i].z)
        //     {
        //         // upload the point and texture coordinates only for points we have depth data for
        //         std::cout << 
        //     }
        // }

        // std::cout << points.size() << std::endl;
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds{end-start};
        std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

        loop_rate.sleep();
    }
    // Don't exit the program.
    ros::spin();  
    return EXIT_SUCCESS;
    
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
