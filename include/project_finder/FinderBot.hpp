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
 * @file FinderBot.hpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief class declaration for FinderBot
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_FINDERBOT_HPP_
#define INCLUDE_FINDERBOT_HPP_

#include <iostream>
#include <memory>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <project_finder/Pose.hpp>
#include <project_finder/RescuePoint.hpp>
#include <project_finder/Navigation.hpp>
#include <project_finder/Detector.hpp>


#include <opencv4/opencv2/opencv.hpp>


/**
 * @brief FinderBot class which will take all the waypoints and fire exit points
 * and rescue humans
 * 
 */
class FinderBot {

 public:
 /**
  * @brief Construct a new Finder Bot object
  * 
  * @param nh 
  */
    explicit FinderBot(ros::NodeHandle& nh);

    /**
     * @brief Destroy the Finder Bot object
     * 
     */
    ~FinderBot();

    /**
     * @brief entry point method for finder bot
     * 
     */
    void start_rescuing();

    /**
     * @brief Set the rescue points object
     * 
     * @param rescue_points 
     */
    void set_rescue_points(std::vector<RescuePoint>& rescue_points);

 private:
 /**
  * @brief Calls Detect method of Detector class for human detection
  * 
  * @return true, if human detected
  * @return false, if human not detected 
  */
    bool detect_human();

    /**
     * @brief subscribes to camera topic and converts ros img to opencv img
     * 
     * @param img_msg 
     */
    void img_callback(const sensor_msgs::ImageConstPtr& img_msg);

 private:
    ros::NodeHandle nh_;
    cv::Mat opencv_img_;
    std::vector<RescuePoint> rescue_points_;
    std::string camera_topic_;
    Navigation navigate_;
    std::unique_ptr<acme::Detector> human_detector_;
    ros::Subscriber img_sub_;

};

#endif  // INCLUDE_FINDERBOT_HPP_