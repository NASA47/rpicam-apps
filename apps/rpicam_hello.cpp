/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * libcamera_hello.cpp - libcamera "hello world" app.
 */

#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <time.h>

#include "core/rpicam_app.hpp"
#include "core/options.hpp"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

using namespace std::placeholders;



// // Function to convert raw Bayer data to 8-bit RGB
// cv::Mat processBayerData(const uint8_t* bayer_data, int width, int height) {
//     // Create a cv::Mat from the raw Bayer data
//     cv::Mat bayer_image(height, width, CV_8UC1, const_cast<uint8_t*>(bayer_data));

//     // Convert Bayer pattern to RGB (BYR2 corresponds to COLOR_BayerBG2RGB in OpenCV)
//     cv::Mat rgb_image;
//     cv::cvtColor(bayer_image, rgb_image, cv::COLOR_BayerBG2RGB); // Adjust pattern if needed

//     return rgb_image;
// }

// Function to convert raw Bayer data to 8-bit RGB
cv::Mat processBayerData(const uint8_t* bayer_data, int width, int height) {
    // Create a cv::Mat from the raw Bayer data
    cv::Mat bayer_image(height, width, CV_8UC3, const_cast<uint8_t*>(bayer_data));

    // // Convert Bayer pattern to RGB (BYR2 corresponds to COLOR_BayerBG2RGB in OpenCV)
    // cv::Mat rgb_image;
    // cv::cvtColor(bayer_image, rgb_image, cv::COLOR_BayerBG2RGB); // Adjust pattern if needed

    return bayer_image;
}

void event_loop(RPiCamApp &app) {
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image_raw", 10);

    app.OpenCamera();
    app.ConfigureViewfinder();
    app.StartCamera();

    struct timespec boot_ts, epoch_ts;
    clock_gettime(CLOCK_BOOTTIME, &boot_ts);
    clock_gettime(CLOCK_REALTIME, &epoch_ts);
    uint64_t boot_epoch_offset = epoch_ts.tv_sec * 1000000000 + epoch_ts.tv_nsec - (boot_ts.tv_sec * 1000000000 + boot_ts.tv_nsec);

    while (ros::ok()) {
        RPiCamApp::Msg msg = app.Wait();
        if (msg.type == RPiCamApp::MsgType::Timeout) {
            ROS_ERROR("ERROR: Device timeout detected, attempting a restart!!!");
            app.StopCamera();
            app.StartCamera();
            continue;
        }
        if (msg.type == RPiCamApp::MsgType::Quit)
            return;
        else if (msg.type != RPiCamApp::MsgType::RequestComplete)
            throw std::runtime_error("unrecognized message!");

        CompletedRequestPtr &completed_request = std::get<CompletedRequestPtr>(msg.payload);
        auto buffer = completed_request->buffers[app.ViewfinderStream()];
        std::cout << "buffer->planes().size(): " << buffer->planes().size() << std::endl;
        if (buffer->planes().size() >= 1) {
            const auto &plane = buffer->planes()[0];
            uint8_t *bayer_data = static_cast<uint8_t *>(mmap(NULL, plane.length, PROT_READ | PROT_WRITE, MAP_SHARED, plane.fd.get(), 0));

            std::cout << "Buffer FD: " << plane.fd.get() << ", Length: " << plane.length << std::endl;
            std::cout << "Plane count: " << buffer->planes().size() << std::endl;
            if (bayer_data) {
                // Process the Bayer data to get an 8-bit RGB image
                // cv::Mat rgb_image = processBayerData(bayer_data, 1280, 720);

                // Create a ROS Image message
                sensor_msgs::Image img_msg;

                auto sensor_ts = completed_request->metadata.get(controls::SensorTimestamp);
                uint64_t sensor_ts_epoch = *sensor_ts + boot_epoch_offset;
                ros::Time sensor_ros_ts(sensor_ts_epoch / 1000000000, sensor_ts_epoch % 1000000000);
                img_msg.header.stamp = sensor_ros_ts;

                img_msg.height = 720;
                img_msg.width = 1280;
                img_msg.encoding = "rgb8";
                img_msg.is_bigendian = false;
                img_msg.step = img_msg.width * 3;
                // memcpy(img_msg.data.data(), rgb_image.data, img_msg.data.size()); 
                img_msg.data.resize(img_msg.step * img_msg.height);
                memcpy(&img_msg.data[0], bayer_data, img_msg.step * img_msg.height);
                // cv::imshow("test", rgb_image);

                // Publish the image
                pub.publish(img_msg);
            }

            if (bayer_data)
                munmap(bayer_data, plane.length);
        }
    }
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "libcamera_hello", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        ros::start();

	try
	{
		RPiCamApp app;
		Options *options = app.GetOptions();
		if (options->Parse(argc, argv))
		{
			if (options->verbose >= 2)
				options->Print();

			event_loop(app);
		}
	}
	catch (std::exception const &e)
	{
		LOG_ERROR("ERROR: *** " << e.what() << " ***");
                ros::shutdown();
		return -1;
	}

	ros::shutdown();

	return 0;
}
