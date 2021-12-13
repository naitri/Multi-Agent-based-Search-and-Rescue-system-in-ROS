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
 * @author Mayank Joshi
 * @author Naitri Rajyaguru
 * @brief class definition for FinderBot class
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <project_finder/FinderBot.hpp>


FinderBot::FinderBot(const std::string& ns) : namespace_(ns) {
    // initialize send goal state to false
    send_goal_state_ = false;

    // initialize mission accomplished status to false
    mission_accomplished_ = false;

    // initialize mission completed count to 0
    mission_count_ = 0;
}

FinderBot::~FinderBot() {
}

std::string FinderBot::get_namespace() {
    return namespace_;
}

void FinderBot::set_waypoint(Pose waypoint) {
    // set waypoint
    waypoint_ = waypoint;
    // set default destination to waypoint
    destination_ = waypoint_;
}

Pose FinderBot::get_waypoint() {
    return waypoint_;
}

void FinderBot::set_fire_exit(Pose fire_exit) {
    fire_exit_ = fire_exit;
}

Pose FinderBot::get_fire_exit() {
    return fire_exit_;
}

Pose FinderBot::get_destination() {
    return destination_;
}

void FinderBot::update_destination(Pose dest) {
    destination_ = dest;
}

bool FinderBot::get_send_goal_state() {
    return send_goal_state_;
}

void FinderBot::set_send_goal_state(bool send_goal) {
    send_goal_state_ = send_goal;
}

bool FinderBot::get_mission_status() {
    return mission_accomplished_;
}

void FinderBot::set_mission_status(bool status) {
    mission_accomplished_ = status;
}

void FinderBot::set_home_location(Pose home_sweet_home) {
    home_location_ = home_sweet_home;
}

Pose FinderBot::get_home_location() {
    return home_location_;
}

void FinderBot::increment_mission_count() {
    mission_count_ += 1;
}

int FinderBot::get_mission_count() {
    return mission_count_;
}
