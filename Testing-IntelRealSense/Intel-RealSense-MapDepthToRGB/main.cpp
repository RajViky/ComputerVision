// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

///////////////////////////////////////////////////////////
// librealsense tutorial #2 - Accessing multiple streams //
///////////////////////////////////////////////////////////

// First include the librealsense C++ header file
#include <librealsense/rs.hpp>
#include <cstdio>

// Also include GLFW to allow for graphical display
#define GLFW_INCLUDE_GLU
#include <GLFW/glfw3.h>


int main() try
{
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    rs::log_to_console(rs::log_severity::warn);
    // Create a context object. This object owns the handles to all connected realsense devices.
    rs::context ctx;
    printf("There are %d connected RealSense devices.\n", ctx.get_device_count());
    if(ctx.get_device_count() == 0) return EXIT_FAILURE;

    // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    rs::device * dev = ctx.get_device(0);
    printf("\nUsing device 0, an %s\n", dev->get_name());
    printf("    Serial number: %s\n", dev->get_serial());
    printf("    Firmware version: %s\n", dev->get_firmware_version());

    // Configure depth and color to run with the device's preferred settings
    dev->enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev->enable_stream(rs::stream::color, rs::preset::best_quality);
    dev->start();

    // Open a GLFW window to display our output
    glfwInit();
    GLFWwindow * win2 = glfwCreateWindow(1280, 960, "Data for SLAM", nullptr, nullptr);

    while(!glfwWindowShouldClose(win2))
    {
        //FIRST
        glfwMakeContextCurrent(win2);
        // Wait for new frame data
        glfwPollEvents();
        dev->wait_for_frames();

        // Retrieve our images
        const uint16_t * depth_image = (const uint16_t *)dev->get_frame_data(rs::stream::depth);

        // Retrieve camera parameters for mapping between depth and color
        rs::intrinsics depth_intrin = dev->get_stream_intrinsics(rs::stream::depth);
        rs::extrinsics depth_to_color = dev->get_extrinsics(rs::stream::depth, rs::stream::color);
        rs::intrinsics color_intrin = dev->get_stream_intrinsics(rs::stream::color);
        float scale = dev->get_depth_scale();


        char greyPixels[640 * 480];
        for (unsigned int i = 0; i < 640 * 480; i++) {
            greyPixels[i] = 0x00;
        }
        for(int dy=0; dy<depth_intrin.height; ++dy)
        {
            for(int dx=0; dx<depth_intrin.width; ++dx)
            {
                // Retrieve the 16-bit depth value and map it into a depth in meters
                uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
                float depth_in_meters = depth_value * scale;


                // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                rs::float2 depth_pixel = {(float)dx, (float)dy};
                rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
                rs::float3 color_point = depth_to_color.transform(depth_point);
                rs::float2 color_pixel = color_intrin.project(color_point);

                // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
                if(cx >= 0 && cy >= 0 && cx < color_intrin.width && cy < color_intrin.height)
                {
                    greyPixels[cy*640+cx] = (char)(depth_in_meters * 128);
                }
            }
        }


        //SECOND
        glfwMakeContextCurrent(win2);
        glClear(GL_COLOR_BUFFER_BIT);
        glPixelZoom(1, -1);

        // Display depth data by linearly mapping depth between 0 and 2 meters to the red channel
        glRasterPos2f(-1, 1);
        glPixelTransferf(GL_RED_SCALE, 0xFFFF * dev->get_depth_scale() / 2.0f);
        glDrawPixels(640, 480, GL_RED, GL_UNSIGNED_SHORT, dev->get_frame_data(rs::stream::depth));
        glPixelTransferf(GL_RED_SCALE, 1.0f);


        glRasterPos2f(0, 0);
        glDrawPixels(640, 480, GL_LUMINANCE, GL_UNSIGNED_BYTE, greyPixels);

        // Display color image as RGB triples
        glRasterPos2f(-1, 0);
        glDrawPixels(640, 480, GL_RGB, GL_UNSIGNED_BYTE, dev->get_frame_data(rs::stream::color));


        glfwSwapBuffers(win2);
    }

    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    // Method calls against librealsense objects may throw exceptions of type rs::error
    printf("rs::error was thrown when calling %s(%s):\n", e.get_failed_function().c_str(), e.get_failed_args().c_str());
    printf("    %s\n", e.what());
    return EXIT_FAILURE;
}
