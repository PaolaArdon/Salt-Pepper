#include <iostream>
//#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <queue>

int main(int argc, char *argv[]) {
    if(argc != 3) {
        std::cout << "usage: " << argv[0] << " <source image> <method: 0 or 1>\n";
        return -1;
    }

    cv::Mat img_object_t = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    if( !img_object_t.data ) { std::cout << "Err: reading object image failed...\n";}

    cv::VideoCapture cap(0);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    
    if(!cap.isOpened())
        return -1;
    
    char* method = argv[2];

    std::vector<cv::KeyPoint> keypoints_object, keypoints_scene;
    cv::Mat descriptors_object, descriptors_scene;

    cv::ORB orb;

    int minHessian = 50;
    cv::SurfFeatureDetector detector(minHessian);
    cv::SurfDescriptorExtractor extractor;
//    cv::OrbFeatureDetector detector;
//    cv::OrbDescriptorExtractor extractor;

    //-- object
//    if( method == 0 ) { //-- ORB
//        orb.detect(img_object, keypoints_object);
//        //cv::drawKeypoints(img_object, keypoints_object, img_object, cv::Scalar(0,255,255));
//        //cv::imshow("template", img_object);
//
//        orb.compute(img_object, keypoints_object, descriptors_object);
//    } else { //-- SURF test
//        detector.detect(img_object, keypoints_object);
//        extractor.compute(img_object, keypoints_object, descriptors_object);
//    }

//    if(descriptors_object.type() != CV_32F)
//        descriptors_object.convertTo(descriptors_object, CV_32F);

    bool tmp = 0;
    cv::Mat img_object;
    
    for(;;) {
        
        cv::Mat frame;
        cap >> frame;
        
        cv::Mat img_scene = cv::Mat(frame.size(), CV_8UC1);
        cv::cvtColor(frame, img_scene, cv::COLOR_RGB2GRAY);

        if( method == 0 ) { //-- ORB
            orb.detect(img_scene, keypoints_scene);
            orb.compute(img_scene, keypoints_scene, descriptors_scene);
        } else { //-- SURF
            detector.detect(img_scene, keypoints_scene);
            extractor.compute(img_scene, keypoints_scene, descriptors_scene);
        }
        
        if (tmp == 0) // Only for the first frame 
        {
            tmp = 1;
            img_object = img_scene;
            keypoints_object = keypoints_scene;
            descriptors_object = descriptors_scene;
            continue;
        }

        //-- matching descriptor vectors using FLANN matcher
        cv::FlannBasedMatcher matcher;
        std::vector<cv::DMatch> matches;
        cv::Mat img_matches;
        
        if(!descriptors_object.empty() && !descriptors_scene.empty()) {
            matcher.match(descriptors_object, descriptors_scene, matches);
            std::vector< std::queue<cv::KeyPoint> > trajectory(matches.size());
            for (int i = 0; i < matches.size(); i++) {
                int id_obj = matches[i].queryIdx;
                int id_scn = matches[i].imgIdx;
                trajectory[i].push_back(keypoints_scene[id_scn]);
                if (trajectory[i].size()>30) {
                    trajectory[i].pop_front();
                }
            }
            
            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min idstance between keypoints
            for( int i = 0; i < descriptors_object.rows; i++)
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }
            //printf("-- Max dist : %f \n", max_dist );
            //printf("-- Min dist : %f \n", min_dist );
            //-- Draw only good matches (i.e. whose distance is less than 3*min_dist)
            std::vector< cv::DMatch >good_matches;

//            for( int i = 0; i < descriptors_object.rows; i++ )
//            { if( matches[i].distance < 3*min_dist )
//                { good_matches.push_back( matches[i]); }
//            }
            for( int i = 0; i < descriptors_object.rows; i++ )
            { if( matches[i].distance <= fmax(2*min_dist, 0.02) )
            { good_matches.push_back( matches[i]); }
            }

//            cv::drawMatches(img_object, keypoints_object, img_scene, keypoints_scene, \
//                    good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
//                    std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
            

            //-- localize the object
            std::vector<cv::Point2f> obj;
            std::vector<cv::Point2f> scene;

            for( size_t i = 0; i < good_matches.size(); i++) {
                //-- get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
            }
            
            img_object = img_scene;
            keypoints_object = keypoints_scene;
            descriptors_object = descriptors_scene;
            
//            if( !obj.empty() && !scene.empty() && good_matches.size() >= 4) {
//                cv::Mat H = cv::findHomography( obj, scene, cv::RANSAC );
//
//                //-- get the corners from the object to be detected
//                std::vector<cv::Point2f> obj_corners(4);
//                obj_corners[0] = cv::Point(0,0);
//                obj_corners[1] = cv::Point(img_object.cols,0);
//                obj_corners[2] = cv::Point(img_object.cols,img_object.rows);
//                obj_corners[3] = cv::Point(0,img_object.rows);
//
//                std::vector<cv::Point2f> scene_corners(4);
//
//                cv::perspectiveTransform( obj_corners, scene_corners, H);
//
//                //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//                cv::line( img_matches, \
//                        scene_corners[0] + cv::Point2f(img_object.cols, 0), \
//                        scene_corners[1] + cv::Point2f(img_object.cols, 0), \
//                        cv::Scalar(0,255,0), 4 );
//                cv::line( img_matches, \
//                        scene_corners[1] + cv::Point2f(img_object.cols, 0), \
//                        scene_corners[2] + cv::Point2f(img_object.cols, 0), \
//                        cv::Scalar(0,255,0), 4 );
//                cv::line( img_matches, \
//                        scene_corners[2] + cv::Point2f(img_object.cols, 0), \
//                        scene_corners[3] + cv::Point2f(img_object.cols, 0), \
//                        cv::Scalar(0,255,0), 4 );
//                cv::line( img_matches, \
//                        scene_corners[3] + cv::Point2f(img_object.cols, 0), \
//                        scene_corners[0] + cv::Point2f(img_object.cols, 0), \
//                        cv::Scalar(0,255,0), 4 );
//
//            }
        }
        cv::imshow("match result", img_matches);

        if(cv::waitKey(30) >= 0) break;
    }

    return 0;
}
