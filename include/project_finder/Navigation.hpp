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
 * @file Navigation.hpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief Navigation class declaration for Project Finder
 * @version 0.1
 * 
 * @copyright MIT License
 * 
 */

#ifndef INCLUDE_NAVIGATION_HPP_
#define INCLUDE_NAVIGATION_HPP_


#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>

#include <project_finder/Pose.hpp>
#include <project_finder/FinderBot.hpp>
#include <project_finder/Detector.hpp>





typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
 MoveBaseClient;

/**
 * @brief Navigation class
 * responsible for all movement and navigates the finderbot
 * 
 */
class Navigation {
 public:
 /**
  * @brief Construct a new Navigation object
  * 
  */
    explicit Navigation(ros::NodeHandle nh, std::vector<FinderBot>& robots);

    /**
     * @brief Destroy the Navigation object
     * 
     */
    ~Navigation();

   /**
    * @brief calls sendGoal method of MoveBaseClient
    * 
    * @param dest_location 
    * @return true 
    * @return false 
    */
    void move_bots();
    static move_base_msgs::MoveBaseGoal get_destination_goal(
      std::shared_ptr<FinderBot>& robot);

 private:
  void initialize_client(std::shared_ptr<FinderBot>& robot,
  std::unique_ptr<MoveBaseClient>& robot_client);
   bool detect_human();
   void update_robot(std::unique_ptr<MoveBaseClient>& robot_client,
    std::shared_ptr<FinderBot>& robot);
   void send_goal(std::unique_ptr<MoveBaseClient>& robot_client,
    std::shared_ptr<FinderBot>& robot,  
    move_base_msgs::MoveBaseGoal& robot_goal);
   bool mission_impossible_complete();

 private:
  std::shared_ptr<FinderBot> robot1_;
  std::shared_ptr<FinderBot> robot2_;
  std::shared_ptr<FinderBot> robot3_;
  std::shared_ptr<FinderBot> robot4_;
  std::shared_ptr<FinderBot> robot5_;
  std::shared_ptr<FinderBot> robot6_;
  std::shared_ptr<FinderBot> robot7_;
  std::shared_ptr<FinderBot> robot8_;
  std::shared_ptr<FinderBot> robot9_;
  std::shared_ptr<FinderBot> robot10_;
  std::shared_ptr<FinderBot> robot11_;
  std::shared_ptr<FinderBot> robot12_;
  std::shared_ptr<FinderBot> robot13_;
  std::shared_ptr<FinderBot> robot14_;
  std::shared_ptr<FinderBot> robot15_;
  std::shared_ptr<FinderBot> robot16_;
  std::shared_ptr<FinderBot> robot17_;
  std::shared_ptr<FinderBot> robot18_;
  std::shared_ptr<FinderBot> robot19_;
  std::shared_ptr<FinderBot> robot20_;

  std::unique_ptr<MoveBaseClient> robot1_client_;
  std::unique_ptr<MoveBaseClient> robot2_client_;
  std::unique_ptr<MoveBaseClient> robot3_client_;
  std::unique_ptr<MoveBaseClient> robot4_client_;
  std::unique_ptr<MoveBaseClient> robot5_client_;
  std::unique_ptr<MoveBaseClient> robot6_client_;
  std::unique_ptr<MoveBaseClient> robot7_client_;
  std::unique_ptr<MoveBaseClient> robot8_client_;
  std::unique_ptr<MoveBaseClient> robot9_client_;
  std::unique_ptr<MoveBaseClient> robot10_client_;
  std::unique_ptr<MoveBaseClient> robot11_client_;
  std::unique_ptr<MoveBaseClient> robot12_client_;
  std::unique_ptr<MoveBaseClient> robot13_client_;
  std::unique_ptr<MoveBaseClient> robot14_client_;
  std::unique_ptr<MoveBaseClient> robot15_client_;
  std::unique_ptr<MoveBaseClient> robot16_client_;
  std::unique_ptr<MoveBaseClient> robot17_client_;
  std::unique_ptr<MoveBaseClient> robot18_client_;
  std::unique_ptr<MoveBaseClient> robot19_client_;
  std::unique_ptr<MoveBaseClient> robot20_client_;

  std::unique_ptr<acme::Detector> detector_;
  
  ros::NodeHandle nh_;
  cv::Mat opencv_img_;
};

#endif  // INCLUDE_NAVIGATION_HPP_