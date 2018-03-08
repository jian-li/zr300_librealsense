// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <librealsense/rs.hpp>
#include <iostream>
#include <iomanip>

int main() try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    // Obtain a list of devices currently present on the system
    rs::context ctx;
    int device_count = ctx.get_device_count();   
    if (!device_count) printf("No device detected. Is it plugged in?\n");

    for(int i = 0; i < device_count; ++i)
    {
        // Show the device name and information
        rs::device * dev = ctx.get_device(i);
        std::cout << "Device " << i << " - " << dev->get_name() << ":\n";
        std::cout << " Serial number: " << dev->get_serial() << "\n";
        std::cout << " Firmware version: " << dev->get_firmware_version() << "\n";
        try { std::cout << " USB Port ID: " << dev->get_usb_port_id() << "\n"; } catch (...) {}
        if (dev->supports(rs::capabilities::adapter_board)) std::cout << " Adapter Board Firmware version: " << dev->get_info(rs::camera_info::adapter_board_firmware_version) << "\n";
        if (dev->supports(rs::capabilities::motion_events)) std::cout << " Motion Module Firmware version: " << dev->get_info(rs::camera_info::motion_module_firmware_version) << "\n";

        // Show which options are supported by this device
        std::cout << " Camera info: \n";
        for (int j = RS_CAMERA_INFO_DEVICE_NAME; j < RS_CAMERA_INFO_COUNT; ++j)
        {
            rs::camera_info param = (rs::camera_info)j;
            if (dev->supports(param))
                std::cout << "    " << std::left << std::setw(20) << rs_camera_info_to_string(rs_camera_info(param)) << ": \t" << dev->get_info(param) << std::endl;
        }

        // Show which options are supported by this device
        std::cout << std::setw(55) << " Supported options:" << std::setw(10) << "min"  << std::setw(10) << " max" << std::setw(6) << " step" << std::setw(10) << " default" << std::endl;
        for(int j = 0; j < RS_OPTION_COUNT; ++j)
        {
            rs::option opt = (rs::option)j;
            if(dev->supports_option(opt))
            {
                double min, max, step, def;
                dev->get_option_range(opt, min, max, step, def);
                std::cout   << "    " << std::left << std::setw(50)  << opt << " : " << std::setw(5) << min << "... " << std::setw(12) << max << std::setw(6) << step << std::setw(10) << def << "\n";
            }
        }

        // different sensor extrinsic parameters
        std::cout << "Extrinsic Parameters:" << std::endl;
        rs::extrinsics fe2imu = dev->get_motion_extrinsics_from(rs::stream::fisheye);
        rs::extrinsics depth2fe = dev->get_extrinsics(rs::stream::depth, rs::stream::fisheye);
        rs::extrinsics rgb2fe = dev->get_extrinsics(rs::stream::color, rs::stream::fisheye);

        std::cout << "fisheye to imu extrinsic:" << std::endl;
        std::cout << "   Rotation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << fe2imu.rotation[0] << " " << fe2imu.rotation[3] << " " << fe2imu.rotation[6] << std::endl;
        std::cout << "   " << std::setprecision(6) << fe2imu.rotation[1] << " " << fe2imu.rotation[4] << " " << fe2imu.rotation[7] << std::endl; 
        std::cout << "   " << std::setprecision(6) << fe2imu.rotation[2] << " " << fe2imu.rotation[5] << " " << fe2imu.rotation[8] << std::endl; 
        std::cout << "   Translation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << fe2imu.translation[0] << " " << fe2imu.translation[1] << " " << fe2imu.translation[2] << std::endl;

        std::cout << "Depth to fisheye extrinsic:" << std::endl; 
        std::cout << "   Rotation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << depth2fe.rotation[0] << " " << depth2fe.rotation[3] << " " << depth2fe.rotation[6] << std::endl;
        std::cout << "   " << std::setprecision(6) << depth2fe.rotation[1] << " " << depth2fe.rotation[4] << " " << depth2fe.rotation[7] << std::endl; 
        std::cout << "   " << std::setprecision(6) << depth2fe.rotation[2] << " " << depth2fe.rotation[5] << " " << depth2fe.rotation[8] << std::endl; 
        std::cout << "   Translation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << depth2fe.translation[0] << " " << depth2fe.translation[1] << " " << depth2fe.translation[2] << std::endl;

        std::cout << "RGB to fisheye extrinsic:" << std::endl; 
        std::cout << "   Rotation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << rgb2fe.rotation[0] << " " << rgb2fe.rotation[3] << " " << rgb2fe.rotation[6] << std::endl;
        std::cout << "   " << std::setprecision(6) << rgb2fe.rotation[1] << " " << rgb2fe.rotation[4] << " " << rgb2fe.rotation[7] << std::endl; 
        std::cout << "   " << std::setprecision(6) << rgb2fe.rotation[2] << " " << rgb2fe.rotation[5] << " " << rgb2fe.rotation[8] << std::endl; 
        std::cout << "   Translation: " << std::endl;
        std::cout << "   " << std::setprecision(6) << rgb2fe.translation[0] << " " << rgb2fe.translation[1] << " " << rgb2fe.translation[2] << std::endl;


        // Show which streams are supported by this device
        for(int j = 0; j < RS_STREAM_COUNT; ++j)
        {
            // Determine number of available streaming modes (zero means stream is unavailable) 
            rs::stream strm = (rs::stream)j;
            int mode_count = dev->get_stream_mode_count(strm);
            if(mode_count == 0) continue;

            // Show each available mode for this stream
            std::cout << " Stream " << strm << " - " << mode_count << " modes:\n";
            for(int k = 0; k < mode_count; ++k)
            {
                // Show width, height, format, and framerate, the settings required to enable the stream in this mode
                int width, height, framerate;
                rs::format format;
                dev->get_stream_mode(strm, k, width, height, format, framerate);
                std::cout << "  " << width << "\tx " << height << "\t@ " << framerate << "Hz\t" << format;

                // Enable the stream in this mode so that we can retrieve its intrinsics
                dev->enable_stream(strm, width, height, format, framerate);
                rs::intrinsics intrin = dev->get_stream_intrinsics(strm);

                // Show horizontal and vertical field of view, in degrees
                std::cout << "\t" << std::setprecision(3) << intrin.hfov() << " x " << intrin.vfov() << " degrees, " << std::endl;  
                
                // cout intrinsic and distortion parameters
                std::cout << std::setprecision(6) << " intrinsic parameter = fx:" << intrin.fx << " fy:" << intrin.fy << " cx:" << intrin.ppx << " cy:" << intrin.ppy 
                << " distortion = " << intrin.model() <<  " k1: " << intrin.coeffs[0] << " k2: " << intrin.coeffs[1] << " k2: " 
                << intrin.coeffs[2] << " k3: "  << intrin.coeffs[3] << " k4: "  << intrin.coeffs[4] << std::endl;
            }


            // Some stream mode combinations are invalid, so disable this stream before moving on to the next one
            dev->disable_stream(strm);
        }
    }

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
