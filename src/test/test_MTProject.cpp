/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "../dataStructures.h"
#include "test_matching2D.hpp"

using namespace std;

void evaluate(string det_type, string desc_type)
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // The data path of the dataset
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // The first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // The last file index to load
    int imgFillWidth = 4;  // The number of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // The number of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // The list of data frames that are held in memory at the same time
    bool bVis = false;            // Visualize the results

    string str_kpts, str_prec, str_time_desc, str_time_det;
    double t_desc, t_det;
    ostringstream streamObj;
    
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // Assemble the filenames for the current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // Load the image from the file and convert it to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // Pass the image into the data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if(imgIndex > (dataBufferSize - 1))
            dataBuffer.erase(dataBuffer.begin());
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
    //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // Extract 2D keypoints from the current image.
        vector<cv::KeyPoint> keypoints; 
        // Generate an empty feature list for the current image
        string detectorType = det_type;

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, &t_det, false);
        }
        else
        {
            if(detectorType.compare("HARRIS") == 0)
                detKeypointsHarris(keypoints, imgGray, &t_det, false);
            else
                detKeypointsModern(keypoints, imgGray, detectorType, &t_det, false);
        }

        if(imgIndex == 0)
            str_kpts = str_kpts + "{" + to_string(keypoints.size()) + ", ";
    
        else if(imgIndex == (imgEndIndex - imgStartIndex))
            str_kpts = str_kpts + to_string(keypoints.size()) + "}";
        
        else
            str_kpts = str_kpts + to_string(keypoints.size()) + ", ";

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // Only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            vector<cv::KeyPoint> rect_kpts;
            for(auto &itr : keypoints)
            {
                if(vehicleRect.contains(itr.pt)) 
                    rect_kpts.push_back(itr);
            }
            keypoints.clear();
            keypoints = rect_kpts;
        }

        if(imgIndex == 0)
            str_prec = str_prec + "{" + to_string(keypoints.size()) + ", ";
    
        else if(imgIndex == (imgEndIndex - imgStartIndex))
            str_prec = str_prec + to_string(keypoints.size()) + "}";
        
        else
            str_prec = str_prec + to_string(keypoints.size()) + ", ";

        //// EOF STUDENT ASSIGNMENT

        // optional : limit the number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // There is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // Push the keypoints of the current frame into the end of the data buffer
        (dataBuffer.end() - 1)->keypoints = keypoints;
    //cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT
        // Binary-string descriptors: ORB, BRIEF, BRISK, FREAK, AKAZE, etc.
        // Floating-point descriptors: SIFT, SURF, GLOH, etc.
        cv::Mat descriptors;
        string descriptorType = desc_type;      // BRIEF, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType, &t_desc);
        //// EOF STUDENT ASSIGNMENT

        // Push the descriptors of the current frame into the end of the data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

    //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN: Useful for 
            string descType;                         // DES_BINARY, DES_HOG
            // https://answers.opencv.org/question/10046/feature-2d-feature-matching-fails-with-assert-statcpp/
            descType = (detectorType.compare("SIFT") == 0) || (descriptorType.compare("SIFT")) == 0 ? "DES_HOG" : "DES_BINARY";
            //descType = "DES_BINARY";
            string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
            if(matcherType.compare("MAT_FLANN") == 0)
                selectorType = "SEL_KNN";

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // Store the keypoint matches for the current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

    //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // Visualize the matches between the current and the previous images
            if(imgIndex == (imgEndIndex - imgStartIndex))
                bVis = true;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images (" 
                                    + detectorType + " detector - "
                                    + descriptorType + " descriptor)" ; //descriptorType, detectorType
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
    //cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }
        
        if(imgIndex == 0)
        {
            streamObj << fixed << setprecision(2) << t_det;
            str_time_det = str_time_det + "{" + streamObj.str() + ", ";
            streamObj.str("");
            streamObj << fixed << setprecision(2) << t_desc;
            str_time_desc = str_time_desc + "{" + streamObj.str() + ", ";
            streamObj.str("");
        }
            
        else if(imgIndex == (imgEndIndex - imgStartIndex))
        {
            streamObj << fixed << setprecision(2) << t_det;
            str_time_det = str_time_det + streamObj.str() + "}";
            streamObj.str("");
            streamObj << fixed << setprecision(2) << t_desc;
            str_time_desc = str_time_desc + streamObj.str() + "}";
            streamObj.str("");
        }
            
        else
        {
            streamObj << fixed << setprecision(2) << t_det;
            str_time_det = str_time_det + streamObj.str() + ", ";
            streamObj.str("");
            streamObj << fixed << setprecision(2) << t_desc;
            str_time_desc = str_time_desc + streamObj.str() + ", ";
            streamObj.str("");
        }  
    } // eof loop over all images
    cout << str_kpts << endl;
    cout << str_prec << endl;
    cout << str_time_det << endl;
    cout << str_time_desc << endl;  
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{
    /* OBTAIN THE DETECTOR AND DESCRIPTOR TYPES */
    string detector[] = {"SHITOMASI", "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    string descriptor[] = {"BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    string det_type = "";
    string desc_type = "";

    // Change the value to "true" if the given arguments will be handled
    bool check_args = true;
    bool err_handle = false;
    
//    if(check_args)
//    {
        // Choose a detector type according the passed argument(s)
        if(argc > 1)
        {
            for(int i = 0; i < sizeof(detector) / sizeof(string); i++)
            {
                if(detector[i].compare(argv[1]) == 0)
                {
                    det_type = argv[1];
                    break;
                }
            }
            if(det_type.compare("") == 0)
            {
                cout << "The given detector argument does not exist. The default detector method will be used" << endl;
                det_type = "ORB";
            }      
        }
        else if(argc == 1)
        {
            cout << "No detector and descriptor arguments are passed. The default detector and descriptor methods will be used" << endl;
            det_type = "ORB";
            desc_type = "FREAK";
        }

        // Choose a descriptor type according the passed argument(s)
        if(argc > 2)
        {
            // The SIFT detector is incompatible with the ORB and the AKAZE descriptors
            bool err_sift = det_type.compare("SIFT") == 0 && (descriptor[1].compare(argv[2]) == 0 || descriptor[3].compare(argv[2]) == 0);
            // The AKAZE detector is only compatible with the AKAZE detector
            bool err_akaze = det_type.compare("AKAZE") == 0 && descriptor[3].compare(argv[2]) != 0;
            for(int i = 0; i < sizeof(descriptor) / sizeof(string); i++)
            {
                if(descriptor[i].compare(argv[2]) == 0)
                {
                    if(err_sift && err_handle)
                    {
                        cout << "The given descriptor argument is incompatible with the given detector type. The default descriptor method will be used" << endl;
                        desc_type = descriptor[4]; // SIFT
                        break;
                    }
                    if(err_akaze && err_handle)
                    {
                        cout << "The given descriptor argument is incompatible with the given detector type. The default descriptor method will be used" << endl;
                        desc_type = descriptor[3]; // AKAZE
                        break;
                    }
                    desc_type = argv[2];
                    break;
                }
            }
            if(desc_type.compare("") == 0)
            {
                cout << "The given descriptor argument does not exist. The default descriptor method will be used" << endl;
                desc_type = "FREAK";
            }
        }
        else if(argc == 2)
        {
            cout << "No detector argument is passed. The default descriptor method will be used." << endl;
            // The SIFT detector is incompatible with the ORB and the AKAZE descriptors
            if(det_type.compare("SIFT") == 0)
                desc_type = descriptor[4]; // BRIEF, FREAK, [SIFT]
            // The AKAZE detector is only compatible with the AKAZE detector
            else if(det_type.compare("AKAZE"))
                desc_type = descriptor[3];
            else
                desc_type = "FREAK";
        }
//    }
//    else
//    {
//        det_type = argv[1];
//        desc_type = argv[2];
//    }

    cout << det_type << " detector - " << desc_type << " descriptor (kpts/det_time/desc_time):" << endl;

    evaluate(det_type, desc_type);

    return 0;
}