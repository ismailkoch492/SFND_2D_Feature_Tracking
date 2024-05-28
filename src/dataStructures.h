#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>


struct DataFrame { // Represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // A camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within the camera image
    cv::Mat descriptors; // The keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // The keypoint matches between the previous and the current frame
};


#endif /* dataStructures_h */
