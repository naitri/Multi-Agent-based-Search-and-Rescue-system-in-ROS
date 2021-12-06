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
#include <project_finder/Pose.hpp>

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
    Navigation();

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
    bool move(Pose& dest_location);

 private:

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
    std::unique_ptr<MoveBaseClient> action_server_;
     move_base_msgs::MoveBaseGoal goal_;
};

#endif  // INCLUDE_NAVIGATION_HPP_