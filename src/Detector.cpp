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
 * @file Detector.cpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief Detection class definition for Project Finder
 * @version 0.1
 * 
 * @copyright Copyright (c) 2021
 * 
 */


#include <project_finder/Detector.hpp>
#include <ros/package.h>



acme::Detector::Detector(double conf, const std::vector<std::string> &classes) {
    /// initializes default parameter and creates the model network
    InitModel(conf, classes);

    /// run the model once before actually performing detection on video frame
    WarmUp();
}
acme::Detector::~Detector() {}

std::vector<acme::Detection> acme::Detector::Detect(const cv::Mat& frame) {
    /// create a vector to store Detection objects
    std::vector<acme::Detection> detections = {};

    /// create a Mat to store the input blob
    cv::Mat b;
    if ( !frame.empty() ) {
        /// create a blob for the model to run a forward pass
        b = cv::dnn::blobFromImage(frame, scale_, size_, mean_, swap_, crop_);

        /// set input for the model
        network_.setInput(b);
        outputs_.clear();

        /// run a forward pass
        network_.forward(outputs_, out_names_);
        cv::Size img_size = frame.size();

        /// process the outputs of the model forward pass
        detections = ProcessNet(img_size);
    }
    return detections;
}

void acme::Detector::SetClasses(const std::vector<std::string> &classes) {
    classes_ = classes;
}

void acme::Detector::SetNmsThresh(const double nms_thresh) {
    nms_thresh_ = nms_thresh;
}

void acme::Detector::SetInputWIdth(const int input_width) {
    input_width_ = input_width;
}

void acme::Detector::SetInputHeight(const int input_height) {
    input_height_ = input_height;
}

void acme::Detector::SetScaleFactor(const double scale_factor) {
    scale_ = scale_factor;
}

void acme::Detector::SetSwapRB(const bool swap_rb) {
    swap_ = swap_rb;
}

void acme::Detector::SetMeanToSubtract(const cv::Scalar &mean) {
    mean_ = mean;
}

void acme::Detector::SetCropImg(const bool crop_img) {
    crop_ = crop_img;
}

void acme::Detector::SetBackend(const int backend) {
    backend_ = backend;
}

void acme::Detector::SetTarget(const int target) {
    target_ = target;
}

void acme::Detector::SetNumChannels(const int num_channels) {
    num_channels_ = num_channels;
}

void acme::Detector::InitModel(double conf, const std::vector<std::string> &c) {
    /// set confidence threshold
    conf_thresh_ = conf;

    /// set classes to detect by the Detector class object
    classes_ = c;

    /// set nms threshold default value
    nms_thresh_ = 0.4;

    /// set input width default value
    input_width_ = 416;

    /// set input height default value
    input_height_ = 416;

    /// set input img size
    size_ = cv::Size(input_width_, input_height_);

    /// set scale factor default value
    scale_ = 0.00392157;

    /// set mean to subtract default value
    mean_ = cv::Scalar();

    /// set swap red and blue channels default value
    swap_ = true;

    /// set random crop img default value
    crop_ = false;

    /// set backend default value for the model
    backend_ = 0;

    /// set target default value for the model
    target_ = 0;

    /// set num of channels default value for the model
    num_channels_ = 3;

    /// set classes default value for the model
    all_classes_ = {"person", "bicycle", "car", "motorbike", "aeroplane",
     "bus", "train", "truck", "boat", "traffic light", "fire hydrant",
      "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
      "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
      "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
      "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
      "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
      "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
      "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
      "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
      "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
      "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
      "scissors", "teddy bear", "hair drier", "toothbrush" };

    std::string ros_path = ros::package::getPath("project_finder");

    /// set weights file path
    std::string weights_path = ros_path + "//data//yolov4-tiny.weights";

    /// set config file path
    std::string config_path = ros_path + "//data//yolov4-tiny.cfg";

    /// create a model network
    network_ = cv::dnn::readNet(weights_path, config_path);

    /// set network backend using the set backend value
    network_.setPreferableBackend(backend_);

    /// set network target using the set target value
    network_.setPreferableTarget(target_);

    /// get output layer names for the model
    out_names_ = network_.getUnconnectedOutLayersNames();
}

void acme::Detector::WarmUp() {
    /// create a temporary Mat object
    cv::Mat temp = cv::Mat::zeros(cv::Size(416, 416), CV_8UC3);

    /// run Detector on the temporary Mat as a warmup
    Detect(temp);
}

std::vector<acme::Detection> acme::Detector::ProcessNet(const cv::Size &s) {
    /// create a variable to store vector of Detection objects
    std::vector<acme::Detection> detections;

    /// create a variable to store vector of detected class name
    std::vector<std::string> class_names;

    /// create a variable to store vector of detected confidences
    std::vector<float> confidences;

    /// create a variable to store vector of detected bbox
    std::vector<cv::Rect> bboxes;

    /// create a variable to store vector of indices for NMS
    std::vector<int> indices;

    for ( cv::Mat &output : outputs_ ) {
        /// cast the Mat data into float data
        auto *data = reinterpret_cast<float*>(output.data);
        for ( int i = 0 ; i < output.rows ; i++, data += output.cols ) {
            /// get x center of detected bbox
            int x_center =  static_cast<int>(data[0]*s.width);

            /// get y center of detected bbox
            int y_center = static_cast<int>(data[1]*s.height);

            /// get scores for the output
            cv::Mat scores = output.row(i).colRange(5, output.cols);

            /// create a variable to store the detected class id
            cv::Point class_id_point;

            /// create a variable to store the detected bbox confidence
            double confidence;

            /// perform minMaxLoc to get global min n max in scores
            cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);
            if ( confidence > conf_thresh_ ) {
                /// get detected class id
                int class_id = class_id_point.x;

                /// get detected class name
                std::string cl4ss = all_classes_[class_id];

                /// find if class name is in the classes to detected vector
                auto iter = std::find(classes_.begin(), classes_.end(), cl4ss);
                if ( iter != classes_.end() ) {
                    x_center = std::max(0, x_center);
                    y_center =  std::max(0, y_center);

                    /// get detected bbox width
                    int width = static_cast<int>(data[2]*s.width);

                    /// get detected bbox height
                    int height = static_cast<int>(data[3]*s.height);

                    /// calculate x - top left coordinate
                    int x_left = x_center - width / 2;

                    /// calculate y - top left coordinate
                    int y_top = y_center - height / 2;

                    /// create a cv::Rect object
                    cv::Rect bbox(x_left, y_top, width, height);

                    /// store the class names in a vector
                    class_names.push_back(cl4ss);

                    /// store the confidences in a vector
                    confidences.push_back(static_cast<float>(confidence));

                    /// store the bbox in a vector
                    bboxes.push_back(bbox);
                }
            }
        }
    }

    /// perform non maximum suppression
    cv::dnn::NMSBoxes(bboxes, confidences, conf_thresh_, nms_thresh_, indices);

    for ( int index : indices ) {
        cv::Rect bbox = bboxes[index];
        double conf = static_cast<double>(confidences[index]);
        std::string name = class_names[index];

        /// create Detection object
        Detection box(bbox, conf, name);

        /// store the Detection objects in a vector
        detections.push_back(box);
    }
    return detections;
}
