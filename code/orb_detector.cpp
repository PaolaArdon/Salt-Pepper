//
//  test_CVcapture.cpp
//  
//
//  Created by Songyou Peng on 04/08/16.
//
//

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

using namespace cv;

int main(int argc, char** argv)
{
    cv::VideoCapture cap;
    
    
    char imagename[200];
    int n(0);
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(1))
        return 0;
    
    std::vector<KeyPoint> kp;
    cv::ORB orb(1000, 1.2f, 8, 31, 0, 2, 31);
//    OrbFeatureDetector detector(700, 1.2f, 8, 31, 0, 2, 31);
//    SiftFeatureDetector detector(700);
    for(;;)
    {
        cv::Mat frame;
        cap >> frame;

        if( frame.empty() ) break; // end of video stream
        
        if( cv::waitKey(10) == 27 ) break; // stop capturing by pressing ESC
        
        
        orb.detect(frame, kp);
//        detector.detect(frame,kp);
        Mat out;
        drawKeypoints(frame, kp, out, Scalar(0,255,0));
        
        cv::imshow("ORB test", out);

    }
    return 0;
}