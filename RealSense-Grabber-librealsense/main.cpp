#include <librealsense/rs.hpp>

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>

#include <GLFW/glfw3.h>

#include <sstream>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>

inline void make_depth_histogram(uint8_t rgb_image[640*480*3], const uint16_t depth_image[], int width, int height)
{
    static uint32_t histogram[0x10000];
    memset(histogram, 0, sizeof(histogram));

    for(int i = 0; i < width*height; ++i) ++histogram[depth_image[i]];
    for(int i = 2; i < 0x10000; ++i) histogram[i] += histogram[i-1]; // Build a cumulative histogram for the indices in [1,0xFFFF]
    for(int i = 0; i < width*height; ++i)
    {
        if(uint16_t d = depth_image[i])
        {
            int f = histogram[d] * 255 / histogram[0xFFFF]; // 0-255 based on histogram location
            rgb_image[i*3 + 0] = 255 - f;
            rgb_image[i*3 + 1] = 0;
            rgb_image[i*3 + 2] = f;
        }
        else
        {
            rgb_image[i*3 + 0] = 20;
            rgb_image[i*3 + 1] = 5;
            rgb_image[i*3 + 2] = 0;
        }
    }
}

////////////////////////
// Image display code //
////////////////////////

class texture_buffer
{
    GLuint texture;
    double last_timestamp;
    std::vector<uint8_t> rgb;

    int fps, num_frames;
    double next_time;

public:
    texture_buffer() : texture(), last_timestamp(-1), fps(), num_frames(), next_time(1000) {}

    GLuint get_gl_handle() const { return texture; }

    void upload(const void * data, int width, int height, rs::format format, int stride = 0)
    {
        // If the frame timestamp has changed since the last time show(...) was called, re-upload the texture
        if(!texture) glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        stride = stride == 0 ? width : stride;
        glPixelStorei(GL_UNPACK_ROW_LENGTH, stride);
        switch(format)
        {
        case rs::format::any:
        throw std::runtime_error("not a valid format");
        case rs::format::z16:
        case rs::format::disparity16:
            rgb.resize(stride * height * 3);
            make_depth_histogram(rgb.data(), reinterpret_cast<const uint16_t *>(data), stride, height);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, stride, height, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());

            break;
        case rs::format::xyz32f:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_FLOAT, data);
            break;
        case rs::format::yuyv: // Display YUYV by showing the luminance channel and packing chrominance into ignored alpha channel
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE_ALPHA, GL_UNSIGNED_BYTE, data);
            break;
        case rs::format::rgb8: case rs::format::bgr8: // Display both RGB and BGR by interpreting them RGB, to show the flipped byte ordering. Obviously, GL_BGR could be used on OpenGL 1.2+
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            break;
        case rs::format::rgba8: case rs::format::bgra8: // Display both RGBA and BGRA by interpreting them RGBA, to show the flipped byte ordering. Obviously, GL_BGRA could be used on OpenGL 1.2+
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            break;
        case rs::format::y8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,  width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, data);
            break;
        case rs::format::y16:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_SHORT, data);
            break;
        case rs::format::raw8:
            glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, data);
            break;
        case rs::format::raw10:
            // Visualize Raw10 by performing a naive downsample. Each 2x2 block contains one red pixel, two green pixels, and one blue pixel, so combine them into a single RGB triple.
            rgb.clear(); rgb.resize(width/2 * height/2 * 3);
            auto out = rgb.data(); auto in0 = reinterpret_cast<const uint8_t *>(data), in1 = in0 + width*5/4;
            for(int y=0; y<height; y+=2)
            {
                for(int x=0; x<width; x+=4)
                {
                    *out++ = in0[0]; *out++ = (in0[1] + in1[0]) / 2; *out++ = in1[1]; // RGRG -> RGB RGB
                    *out++ = in0[2]; *out++ = (in0[3] + in1[2]) / 2; *out++ = in1[3]; // GBGB
                    in0 += 5; in1 += 5;
                }
                in0 = in1; in1 += width*5/4;
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width/2, height/2, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb.data());
            break;
        }
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, 0);

    }

    void upload(rs::device & dev, rs::stream stream)
    {
        assert(dev.is_stream_enabled(stream));

        const double timestamp = dev.get_frame_timestamp(stream);
        if(timestamp != last_timestamp)
        {
            upload(dev.get_frame_data(stream), dev.get_stream_width(stream), dev.get_stream_height(stream), dev.get_stream_format(stream));
            last_timestamp = timestamp;

            ++num_frames;
            if(timestamp >= next_time)
            {
                fps = num_frames;
                num_frames = 0;
                next_time += 1000;
            }
        }
    }

    void upload(rs::frame& frame)
    {
        const double timestamp = frame.get_timestamp();
        if(timestamp != last_timestamp)
        {
            upload(frame.get_data(), frame.get_width(), frame.get_height(), frame.get_format(), frame.get_stride());
            last_timestamp = timestamp;

            ++num_frames;
            if(timestamp >= next_time)
            {
                fps = num_frames;
                num_frames = 0;
                next_time += 1000;
            }
        }
    }

    void show(float rx, float ry, float rw, float rh) const
    {
        glBindTexture(GL_TEXTURE_2D, texture);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 0); glVertex2f(rx,    ry   );
        glTexCoord2f(1, 0); glVertex2f(rx+rw, ry   );
        glTexCoord2f(1, 1); glVertex2f(rx+rw, ry+rh);
        glTexCoord2f(0, 1); glVertex2f(rx,    ry+rh);
        glEnd();
        glDisable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    void print(int x, int y, const char * text)
    {
        char buffer[20000]; // ~100 chars

        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(2, GL_FLOAT, 16, buffer);
        //glDrawArrays(GL_QUADS, 0, 4*stb_easy_font_print((float)x, (float)y, (char *)text, nullptr, buffer, sizeof(buffer)));
        glDisableClientState(GL_VERTEX_ARRAY);
    }

    void show(rs::device & dev, rs::stream stream, int rx, int ry, int rw, int rh)
    {
        if(!dev.is_stream_enabled(stream)) return;

        upload(dev, stream);

        int width = dev.get_stream_width(stream), height = dev.get_stream_height(stream);
        float h = (float)rh, w = (float)rh * width / height;
        if(w > rw)
        {
            float scale = rw/w;
            w *= scale;
            h *= scale;
        }

        show(rx + (rw - w)/2, ry + (rh - h)/2, w, h);
        std::ostringstream ss; ss << stream << ": " << width << " x " << height << " " << dev.get_stream_format(stream) << " (" << fps << "/" << dev.get_stream_framerate(stream) << ")" << ", F#: " << dev.get_frame_number(stream);
        glColor3f(0,0,0);
        //draw_text(rx+9, ry+17, ss.str().c_str());
        glColor3f(1,1,1);
        //draw_text(rx+8, ry+16, ss.str().c_str());
    }

    void show(rs::stream stream, rs::format format, int stream_framerate, unsigned long long frame_number, double timestamp, int rx, int ry, int rw, int rh, int width, int height)
    {
        show(rx, ry, rw, rh, width, height);
        if (frame_number != 0)
        {
            std::ostringstream ss; ss << stream << ": " << width << " x " << height << " " << format << " (" << fps << "/" << stream_framerate << ")" << ", F#: " << frame_number << ", TS: " << timestamp;
            glColor3f(0,0,0);
            //draw_text(rx+9, ry+17, ss.str().c_str());
            glColor3f(1,1,1);
            //draw_text(rx+8, ry+16, ss.str().c_str());
        }
    }

    void show(int rx, int ry, int rw, int rh, int width, int height)
    {
        float h = (float)rh, w = (float)rh * width / height;
        if (w > rw)
        {
            float scale = rw / w;
            w *= scale;
            h *= scale;
        }

        show(rx + (rw - w) / 2, ry + (rh - h) / 2, w, h);
    }

    void show(const void * data, int width, int height, rs::format format, const std::string & caption, int rx, int ry, int rw, int rh)
    {
        if(!data) return;

        upload(data, width, height, format);

        float h = (float)rh, w = (float)rh * width / height;
        if(w > rw)
        {
            float scale = rw/w;
            w *= scale;
            h *= scale;
        }

        show(rx + (rw - w)/2, ry + (rh - h)/2, w, h);

        std::ostringstream ss; ss << caption << ": " << width << " x " << height << " " << format;
        glColor3f(0,0,0);
        //draw_text(rx+9, ry+17, ss.str().c_str());
        glColor3f(1,1,1);
        //draw_text(rx+8, ry+16, ss.str().c_str());
    }
};


texture_buffer buffers[6];

#pragma pack(push, 1)
struct rgb_pixel
{
    uint8_t r,g,b;
};
#pragma pack(pop)

int main(int argc, char * argv[]) try
{
    rs::log_to_console(rs::log_severity::warn);
    //rs::log_to_file(rs::log_severity::debug, "librealsense.log");

    rs::context ctx;
    if(ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    dev.enable_stream(rs::stream::depth, rs::preset::best_quality);
    dev.enable_stream(rs::stream::color, rs::preset::best_quality);
    dev.start();

    // Open a GLFW window
    glfwInit();
    std::ostringstream ss; ss << "CPP Image Alignment Example (" << dev.get_name() << ")";
    GLFWwindow * win = glfwCreateWindow(dev.is_stream_enabled(rs::stream::infrared2) ? 1920 : 1280, 960, ss.str().c_str(), 0, 0);
    glfwMakeContextCurrent(win);

    while (!glfwWindowShouldClose(win))
    {
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // Clear the framebuffer
        int w,h;
        glfwGetFramebufferSize(win, &w, &h);
        glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images
        glPushMatrix();
        glfwGetWindowSize(win, &w, &h);
        glOrtho(0, w, h, 0, -1, +1);
        int s = w / (dev.is_stream_enabled(rs::stream::infrared2) ? 3 : 2);
        buffers[0].show(dev, rs::stream::color, 0, 0, s, h-h/2);
        buffers[1].show(dev, rs::stream::color_aligned_to_depth, s, 0, s, h-h/2);
        buffers[2].show(dev, rs::stream::depth_aligned_to_color, 0, h/2, s, h-h/2);
        buffers[3].show(dev, rs::stream::depth, s, h/2, s, h-h/2);
        glPopMatrix();
        glfwSwapBuffers(win);

        cv::Mat rgb(480, 640, CV_8UC3, (uchar *) dev.get_frame_data(rs::stream::color));
        cv::Mat depth16(480, 640, CV_16U, (uchar *) dev.get_frame_data(rs::stream::depth_aligned_to_color));

        std::string dir = "/tmp/kinect2";
        imwrite( dir +"/depth/tmp-depth.png", depth16 );
        imwrite( dir +"/rgb/tmp-rgb.png", rgb );

        boost::filesystem::rename(dir +"/depth/tmp-depth.png",dir +"/depth/depth.png");
        boost::filesystem::rename(dir +"/rgb/tmp-rgb.png",dir +"/rgb/rgb.png");
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return EXIT_SUCCESS;
}
catch(const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch(const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
