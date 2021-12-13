/******************************************************************************
 * MIT License
 * 
 * Copyright (c) 2021 Mayank Joshi, Naitri Rajyaguru
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 ******************************************************************************/

/**
 * @file main.cpp
 * @author Mayank Joshi
 * @author Naitri Rajyaguru
 * @brief main
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <ros/ros.h>
#include <project_finder/FinderBot.hpp>
#include <project_finder/Navigation.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "finder_bot");
    // set robot namespaces
    std::string robot1_ns = "robot1";
    std::string robot2_ns = "robot2";
    std::string robot3_ns = "robot3";
    std::string robot4_ns = "robot4";
    std::string robot5_ns = "robot5";
    std::string robot6_ns = "robot6";
    std::string robot7_ns = "robot7";
    std::string robot8_ns = "robot8";
    std::string robot9_ns = "robot9";
    std::string robot10_ns = "robot10";
    std::string robot11_ns = "robot11";
    std::string robot12_ns = "robot12";
    std::string robot13_ns = "robot13";
    std::string robot14_ns = "robot14";
    std::string robot15_ns = "robot15";
    std::string robot16_ns = "robot16";
    std::string robot17_ns = "robot17";
    std::string robot18_ns = "robot18";
    std::string robot19_ns = "robot19";
    std::string robot20_ns = "robot20";

    // create FinderBot object and initialize with robot namespace
    FinderBot robot1(robot1_ns);
    FinderBot robot2(robot2_ns);
    FinderBot robot3(robot3_ns);
    FinderBot robot4(robot4_ns);
    FinderBot robot5(robot5_ns);
    FinderBot robot6(robot6_ns);
    FinderBot robot7(robot7_ns);
    FinderBot robot8(robot8_ns);
    FinderBot robot9(robot9_ns);
    FinderBot robot10(robot10_ns);
    FinderBot robot11(robot11_ns);
    FinderBot robot12(robot12_ns);
    FinderBot robot13(robot13_ns);
    FinderBot robot14(robot14_ns);
    FinderBot robot15(robot15_ns);
    FinderBot robot16(robot16_ns);
    FinderBot robot17(robot17_ns);
    FinderBot robot18(robot18_ns);
    FinderBot robot19(robot19_ns);
    FinderBot robot20(robot20_ns);

    // set robot home location
    robot1.set_home_location(Pose(6.0, -9.0));
    robot2.set_home_location(Pose(6.0, -8.0));
    robot3.set_home_location(Pose(6.0, -7.0));
    robot4.set_home_location(Pose(6.0, -6.0));
    robot5.set_home_location(Pose(7, -9));
    robot6.set_home_location(Pose(7, -8));
    robot7.set_home_location(Pose(7, -7));
    robot8.set_home_location(Pose(7, -6));
    robot9.set_home_location(Pose(8, -9));
    robot10.set_home_location(Pose(8, -8));
    robot11.set_home_location(Pose(8, -7));
    robot12.set_home_location(Pose(8, -6));
    robot13.set_home_location(Pose(9, -9.0));
    robot14.set_home_location(Pose(9, -8.0));
    robot15.set_home_location(Pose(9, -7));
    robot16.set_home_location(Pose(9, -6));
    robot17.set_home_location(Pose(10, -9));
    robot18.set_home_location(Pose(10, -8));
    robot19.set_home_location(Pose(10, -7));
    robot20.set_home_location(Pose(10, -6));


    // set robot waypoint
    robot1.set_waypoint(Pose(-11, 2.2));
    robot2.set_waypoint(Pose(-7.0, 8.7));
    robot3.set_waypoint(Pose(-11.0, -1.8));
    robot4.set_waypoint(Pose(-11.0, -6.8));
    robot5.set_waypoint(Pose(-9, -6.8));
    robot6.set_waypoint(Pose(-3.3, -9.1));
    robot7.set_waypoint(Pose(-8.7, -1.7));
    robot8.set_waypoint(Pose(1.57, 4.68));
    robot9.set_waypoint(Pose(4.18, 4.68));
    robot10.set_waypoint(Pose(3.72, -0.56));
    robot11.set_waypoint(Pose(-15.9, 2.15));
    robot12.set_waypoint(Pose(-4.4, 8.7));
    robot13.set_waypoint(Pose(-10.9, 0.04));
    robot14.set_waypoint(Pose(-14.7, -8.8));
    robot15.set_waypoint(Pose(-8.94, -4.9));
    robot16.set_waypoint(Pose(-8.6, 0.43));
    robot17.set_waypoint(Pose(1.86, 1.18));
    robot18.set_waypoint(Pose(4.1, 1.2));
    robot19.set_waypoint(Pose(8.6, 8.45));
    robot20.set_waypoint(Pose(8.67, -3.93));


    // set robot fireexit
    robot1.set_fire_exit(Pose(-16.5, 9.13));
    robot2.set_fire_exit(Pose(-16.6, 9));
    robot3.set_fire_exit(Pose(-16.6, 0.04));
    robot4.set_fire_exit(Pose(-16.2, 0.04));
    robot5.set_fire_exit(Pose(9, -9));
    robot6.set_fire_exit(Pose(7, -8));
    robot7.set_fire_exit(Pose(7, -7));
    robot8.set_fire_exit(Pose(1.32, 9.7));
    robot9.set_fire_exit(Pose(1.32, 9.5));
    robot10.set_fire_exit(Pose(8, -8));
    robot11.set_fire_exit(Pose(-16.5, 9.13));
    robot12.set_fire_exit(Pose(1.3, 9.4));
    robot13.set_fire_exit(Pose(-16.3, 0.05));
    robot14.set_fire_exit(Pose(-16.1, 0.045));
    robot15.set_fire_exit(Pose(9, -7));
    robot16.set_fire_exit(Pose(1.32, 9.42));
    robot17.set_fire_exit(Pose(10, -9));
    robot18.set_fire_exit(Pose(10, -8));
    robot19.set_fire_exit(Pose(1.32, 9.52));
    robot20.set_fire_exit(Pose(10, -6));


    std::vector<FinderBot> robots = {robot1, robot2, robot3, robot4,
     robot5, robot6, robot7, robot8, robot9, robot10, robot11,
     robot12, robot13, robot14,
     robot15, robot16, robot17, robot18, robot19, robot20};

    ros::NodeHandle nh;
    // create Navigation class object and initialize
    // with ros nodehandle and robots
    Navigation navigate(nh, robots);

    // call navigate move_base method
    navigate.move_bots();

    return 0;
}
