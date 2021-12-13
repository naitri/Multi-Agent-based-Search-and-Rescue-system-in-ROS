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
 * @file Pose.hpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief structure for Pose
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef INCLUDE_PROJECT_FINDER_POSE_HPP_
#define INCLUDE_PROJECT_FINDER_POSE_HPP_
/**
 * @brief Structure to store waypoints in ROS format
 * x, y, z - positions
 * roll, pitch, yaw - euler angles
 * qx, qy, qz, qw - quaternion
 * 
 * 
 */
struct Pose {Pose(double x = 0.0, double y = 0.0, double z = 0.0,
double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw =  1.0) :
x(x), y(y), z(z),
roll(roll), pitch(pitch), yaw(yaw),
qx(qx), qy(qy), qz(qz), qw(qw) {}

~Pose() {}

double x, y, z, roll, pitch, yaw, qx, qy, qz, qw;
};
#endif  // INCLUDE_PROJECT_FINDER_POSE_HPP_
