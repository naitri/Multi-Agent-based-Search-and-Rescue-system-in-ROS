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
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <actionlib/client/simple_action_client.h>

#include <memory>

#include <project_finder/Navigation.hpp>

#include <project_finder/FinderBot.hpp>
#include <project_finder/Detector.hpp>
#include <project_finder/Pose.hpp>


double conf = 0.41;
std::vector<std::string> classes = {"person"};
acme::Detector detect_object(conf, classes);
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

cv::Mat img_callback(const sensor_msgs::ImageConstPtr& img_msg) {
    cv::Mat opencv_img_;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    if (cv_ptr->image.cols && cv_ptr->image.rows) {
        opencv_img_ = cv_ptr->image;
    }
    return opencv_img_;
}

TEST(SubscriberTest, testSubscribeMethod) {
    image_transport::ImageTransport it(*nh);
    image_transport::Subscriber img_sub_ =
    it.subscribe("/camera/rgb/image_raw", 1, &img_callback);
    ASSERT_TRUE(img_sub_);
}


TEST(Detector, Detect) {
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



int main(int argc, char** argv) {
  ros::init(argc, argv, "finder_test");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
