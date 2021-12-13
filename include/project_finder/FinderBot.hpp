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
   
   std::string get_namespace();
   void set_waypoint(Pose waypoint);
   void set_fire_exit(Pose fire_exit);
   Pose get_waypoint();
   Pose get_fire_exit();
   void update_destination(Pose dest);
   Pose get_destination();
   void set_send_goal_state(bool send_goal);
   bool get_send_goal_state();
   void set_mission_status(bool status);
   bool get_mission_status();
   void set_home_location(Pose home_sweet_home);
   Pose get_home_location();
   void increment_mission_count();
   int get_mission_count();

 private:
   std::string namespace_;
   Pose waypoint_;
   Pose fire_exit_;
   Pose destination_;
   bool send_goal_state_;
   bool mission_accomplished_;
   Pose home_location_;
   int mission_count_;
   
};

#endif  // INCLUDE_FINDERBOT_HPP_