/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Temperature.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "../../../include/RecognitionMessage.h"

#include <opencv2/core/core.hpp>
#include "Converter.h"


#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg,const sensor_msgs::TemperatureConstPtr& msgRecog);

    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;

};

//ros::Publisher pPosPub;

void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty())
    {

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        
    
        /*
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                            Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                            Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
            tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

            tf::Transform tfTcw(M,V);

            //mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
        */
        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(2);
        poseMSG.pose.position.z = twc.at<float>(1);
        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "VSLAM";
        poseMSG.header.stamp = ros::Time::now();
        //cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        (pPosPub)->publish(poseMSG);

        //mlbLost.push_back(mState==LOST);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();
	bool bReuseMap = false;
    if(argc < 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
	if (!strcmp(argv[3], "true"))
    {
		bReuseMap = true;
	}
   	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR, true, bReuseMap);
    
  //   if (bReuseMap)
		// SLAM.LoadMap("Slam_Map.bin");
    
	ImageGrabber igb(&SLAM);


    ros::NodeHandle nh;
    // ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Temperature> recog_sub(nh, "/recog", 1);
    // TODO: subscribe to the recognition topic
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Temperature> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, recog_sub);
    // TODO: bind recognition message to GrabRGBD
    sync.registerCallback(boost::bind(&ImageGrabber::GrabImage,&igb,_1,_2));
    // Pose broadcaster
    // pPosPub = new ros::Publisher;
    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);
    
        igb.pPosPub = &(PosPub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();


    // Save map
    SLAM.SaveMap("Slam_latest_Map.bin");
    

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::TemperatureConstPtr& msgRecog)
{
    // std::cout << msgRecog->variance << std::endl << msgRecog->temperature << std::endl << msgRecog->header.frame_id << std::endl;
    ORB_SLAM2::RecognitionMessage recog(msgRecog->variance, msgRecog->temperature, msgRecog->header.frame_id);
    // msgRecog->variance > 0 ? recog.flag = true : recog.flag = false;
    // recog.depth = msgRecog->temperature;
    // recog.name = msgRecog->header.frame_id;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw= mpSLAM->TrackMonocular(cv_ptr->image, cv_ptr->header.stamp.toSec(), recog);
    PublishPose(Tcw);
    //usleep(10000);
}


