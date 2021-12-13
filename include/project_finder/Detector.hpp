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
 * @file Detector.hpp
 * @author Phase 2 - Mayank Joshi (driver) and Naitri Rajyaguru (navigator)
 * @brief Detector class declaration for Project Finder
 * @version 0.1
 * 
 * @copyright MIT License
 * 
 */


#ifndef INCLUDE_PROJECT_FINDER_DETECTOR_HPP_
#define INCLUDE_PROJECT_FINDER_DETECTOR_HPP_

#include <memory>
#include <vector>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/dnn/shape_utils.hpp>


namespace acme {
/**
 * @brief Structure to store detector output
 *  
 */
struct Detection {
      Detection(cv::Rect box, double conf, const std::string &n) :
      bbox(box), confidence(conf), name(n) {}
      cv::Rect bbox;
      double confidence;
      std::string name;
};


/**
 * @brief Detector class to get detections on an image
 * 
 */
class Detector {
 public:
    /**
     * @brief Construct a new Detector object
     * 
     * @param confidence
     * @param classes to detect
     */
    Detector(double conf, const std::vector<std::string> &classes);

    /**
     * @brief Destroy the Detector object
     * 
     */
    ~Detector();

      /**
       * @brief Runs Detector on an image and outputs the location of object
       * 
       * @param frame
       * @return vector of Detection object
       */
      std::vector<acme::Detection> Detect(const cv::Mat& frame);

      /**
       * @brief Set the classes to be detected by the detector
       * 
       * @param vector of classes
       */
      void SetClasses(const std::vector<std::string> &classes = {});

      /**
       * @brief Set the nms threshold for model network
       * 
       * @param nms_thresh
       */
      void SetNmsThresh(const double nms_thresh);

      /**
       * @brief Set the input width for the model network
       * 
       * @param input_width
       */
      void SetInputWIdth(const int input_width);

      /**
       * @briefSet the input height for the model network
       * 
       * @param input_height
       */
      void SetInputHeight(const int input_height);

      /**
       * @brief Set the scale factor for the model network
       * 
       * @param scale_factor
       */
      void SetScaleFactor(const double scale_factor);

      /**
       * @brief Set the mean to subtract for the model network
       * 
       * @param mean to subtract
       */
      void SetMeanToSubtract(const cv::Scalar &mean);

      /**
       * @brief Set the swap Red and Blue channel for model network
       * 
       * @param swap_rb
       */
      void SetSwapRB(const bool swap_rb);

      /**
       * @brief Set the random Crop Img for the model network
       * 
       * @param crop_img
       */
      void SetCropImg(const bool crop_img);

      /**
       * @brief Set the Backend value for the model network
       * 
       * @param backend
       */
      void SetBackend(const int backend);

      /**
       * @brief Set the Target value for the model network
       * 
       * @param target
       */
      void SetTarget(const int target);

      /**
       * @brief Set the Num Channels for the model network
       * 
       * @param num_channels
       */
      void SetNumChannels(const int num_channels);

 private:
      /**
       * @brief Run the model network once when initializing the Detector class
       * 
       */
      void WarmUp();

      /**
       * @brief Initialize default model parameters
       * 
       * Required parameters Confidence and classes to detect
       * 
       * @param confidence
       * @param classes to detect
       */
      void InitModel(double conf, const std::vector<std::string> &c);

      /**
       * @brief Process the output of one forward pass by the model network
       * 
       * @param size of the input img
       * @return vector of Detection object
       */
      std::vector<acme::Detection> ProcessNet(const cv::Size &s);

 private:
        double conf_thresh_;
        std::vector<std::string> all_classes_;
        std::vector<std::string> classes_;
        double nms_thresh_;
        double input_width_;
        double input_height_;
        cv::Size size_;
        double scale_;
        cv::Scalar mean_;
        bool swap_;
        bool crop_;
        int backend_;
        int target_;
        int num_channels_;
        cv::dnn::Net network_;
        std::vector<std::string> out_names_;
        std::vector<cv::Mat> outputs_;
};
}  // namespace acme
#endif  // INCLUDE_PROJECT_FINDER_DETECTOR_HPP_
