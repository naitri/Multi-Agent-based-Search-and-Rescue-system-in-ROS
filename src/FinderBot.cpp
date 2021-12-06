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
 * @file FinderBot.cpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief class definition for FinderBot class
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <project_finder/FinderBot.hpp>
#include <memory>

FinderBot::FinderBot(ros::NodeHandle& nh) : nh_(nh) {
}

FinderBot::~FinderBot() {
}

void FinderBot::set_rescue_points(std::vector<RescuePoint>& rescue_points) {
}

void FinderBot::img_callback(const sensor_msgs::ImageConstPtr& img_msg) {
}

bool FinderBot::detect_human() {
return true;
}

void FinderBot::start_rescuing() {
}
