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
 * @author Mayank Joshi
 * @author Naitri Rajyaguru
 * @brief class declaration for FinderBot
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_PROJECT_FINDER_FINDERBOT_HPP_
#define INCLUDE_PROJECT_FINDER_FINDERBOT_HPP_

#include <iostream>
#include <memory>
#include <string>
#include <project_finder/Pose.hpp>


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
  * @param ns
  */
    explicit FinderBot(const std::string& ns);
  /**
     * @brief Destroy the Finder Bot object
     * 
     */
    ~FinderBot();
  /**
    * @brief Get the namespace object
    * 
    * @return std::string 
    */
  std::string get_namespace();
  /**
    * @brief Set the waypoint object
    * 
    * @param waypoint 
    */
  void set_waypoint(Pose waypoint);
  /**
    * @brief Set the fire exit object
    * 
    * @param fire_exit 
    */
  void set_fire_exit(Pose fire_exit);
  /**
    * @brief Get the waypoint object
    * 
    * @return Pose 
    */
  Pose get_waypoint();
  /**
    * @brief Get the fire exit object
    * 
    * @return Pose 
    */
  Pose get_fire_exit();
  /**
   * @brief udpate destination of the robot
    * 
    * @param dest 
    */
  void update_destination(Pose dest);
  /**
    * @brief Get the destination object
    * 
    * @return Pose 
    */
  Pose get_destination();
  /**
    * @brief Set the send goal state object
    * 
    * @param send_goal 
    */
  void set_send_goal_state(bool send_goal);
  /**
    * @brief Get the send goal state object
    * 
    * @return true 
    * @return false 
    */
  bool get_send_goal_state();
  /**
    * @brief Set the mission status object
    * 
    * @param status 
    */
  void set_mission_status(bool status);
  /**
    * @brief Get the mission status object
    * 
    * @return true 
    * @return false 
    */
  bool get_mission_status();
  /**
    * @brief Set the home location object
    * 
    * @param home_sweet_home 
    */
  void set_home_location(Pose home_sweet_home);
    /**
    * @brief Get the home location object
    * 
    * @return Pose 
    */
  Pose get_home_location();
  /**
    * @brief increase robot mission completed count
    * 
    */
  void increment_mission_count();
  /**
    * @brief Get the mission count object
    * 
    * @return int 
    */
  int get_mission_count();

 private:
// variable to store the robot namespace
  std::string namespace_;
// variable to store the robot waypoint
  Pose waypoint_;

  // variable to store the robot fire exit location
  Pose fire_exit_;

  // variable to store the robot destination
  Pose destination_;

  // variable to store the robot send goal status
  bool send_goal_state_;

  // variable to store the robot mission status
  bool mission_accomplished_;

  // variable to store the robot home location
  Pose home_location_;

  // variable to store the robot mission count
  int mission_count_;
};

#endif  // INCLUDE_PROJECT_FINDER_FINDERBOT_HPP_
