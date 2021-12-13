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
 * @file finder_test.cpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief unit test
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <actionlib/client/simple_action_client.h>

#include <memory>

#include <project_finder/Navigation.hpp>
#include <project_finder/FinderBot.hpp>
#include <project_finder/Detector.hpp>
#include <project_finder/Pose.hpp>


double conf = 0.41;
std::vector<std::string> classes = {"person"};
acme::Detector detect_object(0.2, classes);
std::shared_ptr<ros::NodeHandle> nh;


TEST(Detector, structure) {
    std::string class_name = "person";
    acme::Detection temp_object(cv::Rect(1, 1, 1, 1), 0.4, class_name);

    ASSERT_EQ(static_cast<int>(temp_object.bbox.x), 1);
    ASSERT_EQ(static_cast<int>(temp_object.bbox.y), 1);
    ASSERT_EQ(static_cast<int>(temp_object.bbox.width), 1);
    ASSERT_EQ(static_cast<int>(temp_object.bbox.height), 1);
    ASSERT_EQ(temp_object.confidence, 0.4);
    ASSERT_EQ(temp_object.name, class_name);
}

TEST(Detector, detect) {
    cv::Mat img;
    auto output = detect_object.Detect(img);
    ASSERT_EQ(static_cast<int>(output.size()), 0);
}

TEST(Detector, SetClassesToDetect) {
    ASSERT_NO_THROW(detect_object.SetClasses());
}

TEST(Detector, setNmsThresh) {
    ASSERT_NO_THROW(detect_object.SetNmsThresh(0.4));
}

TEST(Detector, setInputWIdth) {
    ASSERT_NO_THROW(detect_object.SetInputWIdth(640));
}

TEST(Detector, setInputHeight) {
    ASSERT_NO_THROW(detect_object.SetInputHeight(480));
}

TEST(Detector, setSwapRB) {
    ASSERT_NO_THROW(detect_object.SetSwapRB(true));
}

TEST(Detector, setCropImg) {
    ASSERT_NO_THROW(detect_object.SetCropImg(false));
}

TEST(Detector, setBackend) {
    ASSERT_NO_THROW(detect_object.SetBackend(0));
}

TEST(Detector, setTarget) {
    ASSERT_NO_THROW(detect_object.SetTarget(0));
}

TEST(Detector, setNumChannels) {
    ASSERT_NO_THROW(detect_object.SetNumChannels(3));
}

TEST(Detector, SetScaleFactor) {
    double sf = 1.0;
    ASSERT_NO_THROW(detect_object.SetScaleFactor(sf));
}

TEST(Detector, SetMeanToSubtract) {
    cv::Scalar mean = cv::Scalar();
    ASSERT_NO_THROW(detect_object.SetMeanToSubtract(mean));
}

TEST(FinderBot, namespace) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    ASSERT_EQ(robot.get_namespace(), ns);
}

TEST(FinderBot, set_waypoint) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose waypoint(1.0, 1.0, 0.0);
    ASSERT_NO_THROW(robot.set_waypoint(waypoint));
}

TEST(FinderBot, get_waypoint) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose waypoint(1.0, 2.0, 0.0);
    robot.set_waypoint(waypoint);

    Pose robot_waypoint = robot.get_waypoint();
    ASSERT_EQ(waypoint.x, robot_waypoint.x);
    ASSERT_EQ(waypoint.y, robot_waypoint.y);
    ASSERT_EQ(waypoint.z, robot_waypoint.z);
    ASSERT_EQ(waypoint.roll, robot_waypoint.roll);
    ASSERT_EQ(waypoint.pitch, robot_waypoint.pitch);
    ASSERT_EQ(waypoint.yaw, robot_waypoint.yaw);
    ASSERT_EQ(waypoint.qx, robot_waypoint.qx);
    ASSERT_EQ(waypoint.qy, robot_waypoint.qy);
    ASSERT_EQ(waypoint.qz, robot_waypoint.qz);
    ASSERT_EQ(waypoint.qw, robot_waypoint.qw);
}

TEST(FinderBot, set_fire_exit) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose fire_exit(1.0, 1.0, 0.0);
    ASSERT_NO_THROW(robot.set_waypoint(fire_exit));
}

TEST(FinderBot, get_fire_exit) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose fire_exit(1.0, 2.0, 0.0);
    robot.set_fire_exit(fire_exit);

    Pose robot_fire_exit = robot.get_fire_exit();
    ASSERT_EQ(fire_exit.x, robot_fire_exit.x);
    ASSERT_EQ(fire_exit.y, robot_fire_exit.y);
    ASSERT_EQ(fire_exit.z, robot_fire_exit.z);
    ASSERT_EQ(fire_exit.roll, robot_fire_exit.roll);
    ASSERT_EQ(fire_exit.pitch, robot_fire_exit.pitch);
    ASSERT_EQ(fire_exit.yaw, robot_fire_exit.yaw);
    ASSERT_EQ(fire_exit.qx, robot_fire_exit.qx);
    ASSERT_EQ(fire_exit.qy, robot_fire_exit.qy);
    ASSERT_EQ(fire_exit.qz, robot_fire_exit.qz);
    ASSERT_EQ(fire_exit.qw, robot_fire_exit.qw);
}

TEST(FinderBot, get_destination) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose waypoint(1.0, 2.0, 0.0);
    robot.set_waypoint(waypoint);

    // by default destination is initialized with the waypoint
    Pose destination = robot.get_destination();
    ASSERT_EQ(destination.x, waypoint.x);
    ASSERT_EQ(destination.y, waypoint.y);
    ASSERT_EQ(destination.z, waypoint.z);
    ASSERT_EQ(destination.roll, waypoint.roll);
    ASSERT_EQ(destination.pitch, waypoint.pitch);
    ASSERT_EQ(destination.yaw, waypoint.yaw);
    ASSERT_EQ(destination.qx, waypoint.qx);
    ASSERT_EQ(destination.qy, waypoint.qy);
    ASSERT_EQ(destination.qz, waypoint.qz);
    ASSERT_EQ(destination.qw, waypoint.qw);
}

TEST(FinderBot, update_destination) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);
    Pose dest(1.0, 2.0, 0.0);
    ASSERT_NO_THROW(robot.update_destination(dest));

    Pose destination = robot.get_destination();
    ASSERT_EQ(destination.x, dest.x);
    ASSERT_EQ(destination.y, dest.y);
    ASSERT_EQ(destination.z, dest.z);
    ASSERT_EQ(destination.roll, dest.roll);
    ASSERT_EQ(destination.pitch, dest.pitch);
    ASSERT_EQ(destination.yaw, dest.yaw);
    ASSERT_EQ(destination.qx, dest.qx);
    ASSERT_EQ(destination.qy, dest.qy);
    ASSERT_EQ(destination.qz, dest.qz);
    ASSERT_EQ(destination.qw, dest.qw);
}

TEST(FinderBot, get_send_goal_state) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_EQ(robot.get_send_goal_state(), false);
}

TEST(FinderBot, set_send_goal_state) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_NO_THROW(robot.set_send_goal_state(true));
    ASSERT_EQ(robot.get_send_goal_state(), true);
}

TEST(FinderBot, get_mission_status) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_EQ(robot.get_mission_status(), false);
}


TEST(FinderBot, set_mission_status) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_NO_THROW(robot.set_mission_status(true));
    ASSERT_EQ(robot.get_mission_status(), true);
}

TEST(FinderBot, set_home_location) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    Pose home_loc(1.0, 2.0);
    ASSERT_NO_THROW(robot.set_home_location(home_loc));
}

TEST(FinderBot, get_home_location) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    Pose home_loc(1.0, 2.0);
    ASSERT_NO_THROW(robot.set_home_location(home_loc));
    Pose robot_home_loc = robot.get_home_location();
    ASSERT_EQ(home_loc.x, robot_home_loc.x);
    ASSERT_EQ(home_loc.y, robot_home_loc.y);
    ASSERT_EQ(home_loc.z, robot_home_loc.z);
    ASSERT_EQ(home_loc.roll, robot_home_loc.roll);
    ASSERT_EQ(home_loc.pitch, robot_home_loc.pitch);
    ASSERT_EQ(home_loc.yaw, robot_home_loc.yaw);
    ASSERT_EQ(home_loc.qx, robot_home_loc.qx);
    ASSERT_EQ(home_loc.qy, robot_home_loc.qy);
    ASSERT_EQ(home_loc.qz, robot_home_loc.qz);
    ASSERT_EQ(home_loc.qw, robot_home_loc.qw);
}

TEST(FinderBot, increment_mission_count) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_NO_THROW(robot.increment_mission_count());
}

TEST(FinderBot, get_mission_count) {
    std::string ns = "awesome_robot1";
    FinderBot robot(ns);

    ASSERT_EQ(robot.get_mission_count(), 0);

    robot.increment_mission_count();
    ASSERT_EQ(robot.get_mission_count(), 1);

    robot.increment_mission_count();
    ASSERT_EQ(robot.get_mission_count(), 2);
}

TEST(Navigation, get_destination_goal) {
    std::string robot_ns = "robot";
    std::shared_ptr<FinderBot> robot = std::make_shared<FinderBot>(robot_ns);
    Pose waypoint(1.0, 2.0, 3.0);
    robot->set_waypoint(waypoint);

    move_base_msgs::MoveBaseGoal goal = Navigation::get_destination_goal(robot);
    ASSERT_EQ(goal.target_pose.header.frame_id, "map");
    ASSERT_EQ(goal.target_pose.pose.position.x, waypoint.x);
    ASSERT_EQ(goal.target_pose.pose.position.y, waypoint.y);
    ASSERT_EQ(goal.target_pose.pose.position.z, waypoint.z);
    ASSERT_EQ(goal.target_pose.pose.orientation.z, waypoint.qz);
    ASSERT_EQ(goal.target_pose.pose.orientation.w, waypoint.qw);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "finder_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
