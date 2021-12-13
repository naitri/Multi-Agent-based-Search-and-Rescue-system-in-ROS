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
 * @file Navigation.cpp
 * @author Mayank Joshi
 * @author Naitri Rajyaguru
 * @brief class definition for Navigation class
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include <project_finder/Navigation.hpp>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
 MoveBaseClient;

Navigation::Navigation(ros::NodeHandle nh, const
std::vector<FinderBot>& robots) {
    nh_ = nh;

    // initialize FinderBot robot 1-20
    robot1_ = std::make_shared<FinderBot>(robots[0]);
    robot2_ = std::make_shared<FinderBot>(robots[1]);
    robot3_ = std::make_shared<FinderBot>(robots[2]);
    robot4_ = std::make_shared<FinderBot>(robots[3]);
    robot5_ = std::make_shared<FinderBot>(robots[4]);
    robot6_ = std::make_shared<FinderBot>(robots[5]);
    robot7_ = std::make_shared<FinderBot>(robots[6]);
    robot8_ = std::make_shared<FinderBot>(robots[7]);
    robot9_ = std::make_shared<FinderBot>(robots[8]);
    robot10_ = std::make_shared<FinderBot>(robots[9]);
    robot11_ = std::make_shared<FinderBot>(robots[10]);
    robot12_ = std::make_shared<FinderBot>(robots[11]);
    robot13_ = std::make_shared<FinderBot>(robots[12]);
    robot14_ = std::make_shared<FinderBot>(robots[13]);
    robot15_ = std::make_shared<FinderBot>(robots[14]);
    robot16_ = std::make_shared<FinderBot>(robots[15]);
    robot17_ = std::make_shared<FinderBot>(robots[16]);
    robot18_ = std::make_shared<FinderBot>(robots[17]);
    robot19_ = std::make_shared<FinderBot>(robots[18]);
    robot20_ = std::make_shared<FinderBot>(robots[19]);

    // initialize move base client for robot 2-20
    initialize_client(robot1_, robot1_client_);
    initialize_client(robot2_, robot2_client_);
    initialize_client(robot3_, robot3_client_);
    initialize_client(robot4_, robot4_client_);
    initialize_client(robot5_, robot5_client_);
    initialize_client(robot6_, robot6_client_);
    initialize_client(robot7_, robot7_client_);
    initialize_client(robot8_, robot8_client_);
    initialize_client(robot9_, robot9_client_);
    initialize_client(robot10_, robot10_client_);
    initialize_client(robot11_, robot11_client_);
    initialize_client(robot12_, robot12_client_);
    initialize_client(robot13_, robot13_client_);
    initialize_client(robot14_, robot14_client_);
    initialize_client(robot15_, robot15_client_);
    initialize_client(robot16_, robot16_client_);
    initialize_client(robot17_, robot17_client_);
    initialize_client(robot18_, robot18_client_);
    initialize_client(robot19_, robot19_client_);
    initialize_client(robot20_, robot20_client_);

    std::vector<std::string> classes = {"person"};
    // initialize Detector with confidence and the classes to detect
    detector_ = std::make_unique<acme::Detector>(0.4, classes);
}

Navigation::~Navigation() {
}

void Navigation::initialize_client(std::shared_ptr<FinderBot>& robot,
  // initialize move base client with the robot namespace
  std::unique_ptr<MoveBaseClient>& robot_client) {
      robot_client  = std::make_unique<MoveBaseClient>(
        "/"+robot->get_namespace()+"/move_base", true);

    // wait for 5 seconds for the server to come up as the map loads
    while (!robot_client->waitForServer(ros::Duration(5.0))) {
    ROS_INFO_STREAM("Waiting for the action server "+robot->get_namespace());
    }
  }

move_base_msgs::MoveBaseGoal Navigation::get_destination_goal(
    const std::shared_ptr<FinderBot>& robot) {
    // variable to store the robot goal location
    move_base_msgs::MoveBaseGoal robot_goal;

    // get destination of the robot
    Pose destination = robot->get_destination();

    // set goal location according to the destination of the robot
    robot_goal.target_pose.header.frame_id = "map";
    robot_goal.target_pose.header.stamp = ros::Time::now();
    robot_goal.target_pose.pose.position.x = destination.x;
    robot_goal.target_pose.pose.position.y = destination.y;
    robot_goal.target_pose.pose.position.z = destination.z;
    robot_goal.target_pose.pose.orientation.z = destination.qz;
    robot_goal.target_pose.pose.orientation.w = destination.qw;

    return robot_goal;
}

void Navigation::send_goal(std::unique_ptr<MoveBaseClient>& robot_client,
std::shared_ptr<FinderBot>& robot, const
move_base_msgs::MoveBaseGoal& robot_goal) {
    // check if send goal command has not been sent
    // and if robot mission is incomplete
    if (!robot->get_send_goal_state() && !robot->get_mission_status()) {
        // send goal to the robot
        robot_client->sendGoal(robot_goal);

        // set robot send goal state as true
        robot->set_send_goal_state(true);
    }
}

bool Navigation::detect_human() {
    try {
        // create an empty img
        cv::Mat empty_img;
        // perform human detection
        auto detections = detector_->Detect(empty_img);
        return true;
    } catch (std::exception& error) {
        return false;
    }
    // NOTE - due to high computation we are not able to fetch video stream
    // of 20 robots and run detector simulatenously
}

void Navigation::update_robot(std::unique_ptr<MoveBaseClient>& robot_client,
 std::shared_ptr<FinderBot>& robot) {
     // get robot namespace
    std::string robot_ns = robot->get_namespace();
    // check robot has reached its goal successfully
    if (robot_client->getState() ==
     actionlib::SimpleClientGoalState::SUCCEEDED) {
         // set robot send goal state as false
        robot->set_send_goal_state(false);

        // increment mission count so that robot can update its destination
        robot->increment_mission_count();

        // upate robot destination according to missions completed
        if (robot->get_mission_count() == 1) {
            // run human detection as the robot has reached waypoint
            if (detect_human()) {
                // print a nice message on console to notify user
                // that robot has rescued human
                ROS_INFO_STREAM(robot_ns+" RESCUED HUMAN");
                // update robot destination to the fire exit location
                robot->update_destination(robot->get_fire_exit());
            } else {
                // since human not found or human detection failed
                // print message that robot did not find human
                // robot going back to home locaiton
                ROS_WARN_STREAM(robot_ns + " found no human, GOING BACK HOME");
                // update robot destination to its home location
                robot->update_destination(robot->get_home_location());
            }
        } else if (robot->get_mission_count() == 2) {
            // print a message to notify user
            // robot has guided human to the nearest exit
            ROS_INFO_STREAM(robot_ns+" GUIDED HUMAN TO FIRE EXIT");
            // update robot location to its home location
            robot->update_destination(robot->get_home_location());
        } else  {
            // print a message to notify user
            // robot has reached back home
            ROS_INFO_STREAM(robot_ns+" REACHED BACK HOME");
            // set robot mission status as true so that it can rest in peace
            robot->set_mission_status(true);
        }
    } else if (robot_client->getState() ==
     actionlib::SimpleClientGoalState::ABORTED ) {
         // if robot stuck, ABORT ALL MISSIONS
        robot->set_mission_status(true);
        ROS_ERROR_STREAM(robot_ns+" STUCK");
    }
}


bool Navigation::mission_impossible_complete() {
    // STOP the execution if all robots mission status is true
    if (robot1_->get_mission_status() && robot2_->get_mission_status() &&
    robot3_->get_mission_status()  && robot5_->get_mission_status() &&
    robot6_->get_mission_status() && robot7_->get_mission_status() &&
    robot8_->get_mission_status()  && robot9_->get_mission_status() &&
    robot10_->get_mission_status()) {
        return true;
    }
    return false;
}

void Navigation::move_bots() {
    // container to store the robot goal position
    move_base_msgs::MoveBaseGoal robot1_goal;
    move_base_msgs::MoveBaseGoal robot2_goal;
    move_base_msgs::MoveBaseGoal robot3_goal;
    move_base_msgs::MoveBaseGoal robot4_goal;
    move_base_msgs::MoveBaseGoal robot5_goal;
    move_base_msgs::MoveBaseGoal robot6_goal;
    move_base_msgs::MoveBaseGoal robot7_goal;
    move_base_msgs::MoveBaseGoal robot8_goal;
    move_base_msgs::MoveBaseGoal robot9_goal;
    move_base_msgs::MoveBaseGoal robot10_goal;
    move_base_msgs::MoveBaseGoal robot11_goal;
    move_base_msgs::MoveBaseGoal robot12_goal;
    move_base_msgs::MoveBaseGoal robot13_goal;
    move_base_msgs::MoveBaseGoal robot14_goal;
    move_base_msgs::MoveBaseGoal robot15_goal;
    move_base_msgs::MoveBaseGoal robot16_goal;
    move_base_msgs::MoveBaseGoal robot17_goal;
    move_base_msgs::MoveBaseGoal robot18_goal;
    move_base_msgs::MoveBaseGoal robot19_goal;
    move_base_msgs::MoveBaseGoal robot20_goal;

    while ( ros::ok() ) {
        // get robot goal destination
        robot1_goal = get_destination_goal(robot1_);
        robot2_goal = get_destination_goal(robot2_);
        robot3_goal = get_destination_goal(robot3_);
        robot4_goal = get_destination_goal(robot4_);
        robot5_goal = get_destination_goal(robot5_);
        robot6_goal = get_destination_goal(robot6_);
        robot7_goal = get_destination_goal(robot7_);
        robot8_goal = get_destination_goal(robot8_);
        robot9_goal = get_destination_goal(robot9_);
        robot10_goal = get_destination_goal(robot10_);
        robot11_goal = get_destination_goal(robot11_);
        robot12_goal = get_destination_goal(robot12_);
        robot13_goal = get_destination_goal(robot13_);
        robot14_goal = get_destination_goal(robot14_);
        robot15_goal = get_destination_goal(robot15_);
        robot16_goal = get_destination_goal(robot16_);
        robot17_goal = get_destination_goal(robot17_);
        robot18_goal = get_destination_goal(robot18_);
        robot19_goal = get_destination_goal(robot19_);
        robot20_goal = get_destination_goal(robot20_);

        // send goal to the robot
        send_goal(robot1_client_, robot1_, robot1_goal);
        send_goal(robot2_client_, robot2_, robot2_goal);
        send_goal(robot3_client_, robot3_, robot3_goal);
        send_goal(robot4_client_, robot4_, robot4_goal);
        send_goal(robot5_client_, robot5_, robot5_goal);
        send_goal(robot6_client_, robot6_, robot6_goal);
        send_goal(robot7_client_, robot7_, robot7_goal);
        send_goal(robot8_client_, robot8_, robot8_goal);
        send_goal(robot9_client_, robot9_, robot9_goal);
        send_goal(robot10_client_, robot10_, robot10_goal);
        send_goal(robot11_client_, robot11_, robot11_goal);
        send_goal(robot12_client_, robot12_, robot12_goal);
        send_goal(robot13_client_, robot13_, robot13_goal);
        send_goal(robot14_client_, robot14_, robot14_goal);
        send_goal(robot15_client_, robot15_, robot15_goal);
        send_goal(robot16_client_, robot16_, robot16_goal);
        send_goal(robot17_client_, robot17_, robot17_goal);
        send_goal(robot18_client_, robot18_, robot18_goal);
        send_goal(robot19_client_, robot19_, robot19_goal);
        send_goal(robot20_client_, robot20_, robot20_goal);

        // update robot only if mission is incomplete
        if (!robot1_->get_mission_status())
        update_robot(robot1_client_, robot1_);
        if (!robot2_->get_mission_status())
        update_robot(robot2_client_, robot2_);
        if (!robot3_->get_mission_status())
        update_robot(robot3_client_, robot3_);
        if (!robot4_->get_mission_status())
        update_robot(robot4_client_, robot4_);
        if (!robot5_->get_mission_status())
        update_robot(robot5_client_, robot5_);
        if (!robot6_->get_mission_status())
        update_robot(robot6_client_, robot6_);
        if (!robot7_->get_mission_status())
        update_robot(robot7_client_, robot7_);
        if (!robot8_->get_mission_status())
        update_robot(robot8_client_, robot8_);
        if (!robot9_->get_mission_status())
        update_robot(robot9_client_, robot9_);
        if (!robot10_->get_mission_status())
        update_robot(robot10_client_, robot10_);
        if (!robot11_->get_mission_status())
        update_robot(robot11_client_, robot11_);
        if (!robot12_->get_mission_status())
        update_robot(robot12_client_, robot12_);
        if (!robot13_->get_mission_status())
        update_robot(robot13_client_, robot13_);
        if (!robot14_->get_mission_status())
        update_robot(robot14_client_, robot14_);
        if (!robot15_->get_mission_status())
        update_robot(robot15_client_, robot15_);
        if (!robot16_->get_mission_status())
        update_robot(robot16_client_, robot16_);
        if (!robot17_->get_mission_status())
        update_robot(robot17_client_, robot17_);
        if (!robot18_->get_mission_status())
        update_robot(robot18_client_, robot18_);
        if (!robot19_->get_mission_status())
        update_robot(robot19_client_, robot19_);
        if (!robot20_->get_mission_status())
        update_robot(robot20_client_, robot20_);

        // exit the function if all robots have completed their tasks
        if (mission_impossible_complete()) {
            return;
        }
    }
}

