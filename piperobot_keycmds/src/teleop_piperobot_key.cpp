#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <turtlesim/action/rotate_absolute.hpp>

#include <signal.h>
#include <stdio.h>

#include <cstdio>
#include <bits/stdc++.h>
#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


// Unsure what this does, taken from turtlesim teleop
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74

using namespace std::chrono_literals;

class RobotKeyboardReader
{
    public:
        RobotKeyboardReader()
        #ifndef _WIN32
        : kfd(0)
        #endif
        {
            #ifndef _WIN32
            // get the console in raw mode
            tcgetattr(kfd, &cooked);
            struct termios raw;
            memcpy(&raw, &cooked, sizeof(struct termios));
            raw.c_lflag &=~ (ICANON | ECHO);
            // Setting a new line, then end of file
            raw.c_cc[VEOL] = 1;
            raw.c_cc[VEOF] = 2;
            tcsetattr(kfd, TCSANOW, &raw);
            #endif
        }
        void readOne(char * c)
        {
            #ifndef _WIN32
            int rc = read(kfd, c, 1);
            if (rc < 0)
            {
                throw std::runtime_error("read failed");
            }
            #else
            for(;;)
            {
                HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
                INPUT_RECORD buffer;
                DWORD events;
                PeekConsoleInput(handle, &buffer, 1, &events);
                if(events > 0)
                {
                    ReadConsoleInput(handle, &buffer, 1, &events);
                    if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
                    {
                        *c = KEYCODE_LEFT;
                        return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
                    {
                        *c = KEYCODE_UP;
                        return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
                    {
                        *c = KEYCODE_RIGHT;
                        return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
                    {
                        *c = KEYCODE_DOWN;
                        return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x45)
                    {
                        *c = KEYCODE_E;
                        return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x52)
                    {
                      *c = KEYCODE_R;
                      return;
                    }
                    else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x54)
                    {
                      *c = KEYCODE_T;
                      return;
                    }
                }
            }
            #endif
        }
        void shutdown()
        {
            #ifndef _WIN32
            tcsetattr(kfd, TCSANOW, &cooked);
            #endif
        }
    private:
        #ifndef _WIN32
        int kfd;
        struct termios cooked;
        #endif
};


class TeleopRobot
{
  public:
    TeleopRobot()
    {
        _nh = rclcpp::Node::make_shared("teleop_robot_node");
        _key_publisher = _nh->create_publisher<std_msgs::msg::String>("/robot_key_cmd", 1);
        _key_timer = _nh->create_wall_timer(500ms, std::bind(&TeleopRobot::timer_callback, this));
        cmd_msg = "0";
    }

    void spin();
    int key_loop();
    void timer_callback();

    rclcpp::Node::SharedPtr _nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _key_publisher;
    rclcpp::TimerBase::SharedPtr _key_timer;
    std::string cmd_msg;

    // void sendGoal(float theta);
    // void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::Ro>
    // void cancelGoal();

};


RobotKeyboardReader input_cmd;


//void quit(int sig)
//{
//    (void)sig;
//    input_cmd.shutdown();
//    rclcpp::shutdown();
//    exit(0);
//}


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // rclcpp::spin(std::make_shared<TeleopRobot>());
    TeleopRobot teleop_robot;
    int trkl = teleop_robot.key_loop();

    input_cmd.shutdown();
    rclcpp::shutdown();
    return 0;
}

void TeleopRobot::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(_nh);
  }
}

void TeleopRobot::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = cmd_msg;
    _key_publisher->publish(message);
    std::string a = "Publishing the message: " + cmd_msg;
    RCLCPP_INFO(_nh->get_logger(), a);

    cmd_msg = "0";
}

int TeleopRobot::key_loop()
{
    char c;

    std::thread{std::bind(&TeleopRobot::spin, this)}.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");

    for(;;)
    {
        // get the next event from the keyboard
        try
        {
            input_cmd.readOne(&c);
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return -1;
        }

        RCLCPP_DEBUG(_nh->get_logger(), "value: 0x%02X\n", c);

        switch(c)
        {
            case KEYCODE_LEFT:
                RCLCPP_INFO(_nh->get_logger(), "LEFT");
                cmd_msg = "LEFT";
                break;
            case KEYCODE_RIGHT:
                RCLCPP_INFO(_nh->get_logger(), "RIGHT");
                cmd_msg = "RIGHT";
                break;
            case KEYCODE_UP:
                RCLCPP_INFO(_nh->get_logger(), "UP");
                cmd_msg = "UP";
                break;
            case KEYCODE_DOWN:
                RCLCPP_INFO(_nh->get_logger(), "DOWN");
                cmd_msg = "DOWN";
                break;
            case KEYCODE_E:
                RCLCPP_DEBUG(_nh->get_logger(), "E");
                cmd_msg = "SLOW";
                break;
            case KEYCODE_T:
                RCLCPP_DEBUG(_nh->get_logger(), "T");
                cmd_msg = "FAST";
                break;
            case KEYCODE_R:
                RCLCPP_DEBUG(_nh->get_logger(), "R");
                cmd_msg = "MEDIUM";
                break;
            // default:
            //     RCLCPP_INFO(_nh->get_logger(), "NO-INPUT");
            //     cmd_msg = "0";
            //     break;
        }
	// auto message = std_msgs::msg::String();
        // message.data = cmd_msg;
        // _key_publisher->publish(message);
    }

    return 0;
}
