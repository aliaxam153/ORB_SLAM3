/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/core.hpp>
#include <string>
#include <cstdlib>
#include <cstdio>

#include"../../../include/System.h"

using namespace std;


    class ImageGrabber
{
public:

    ros::Publisher pub;
    ros::Publisher reward_pub;
    ros::Publisher episode_pub;

    ImageGrabber(ORB_SLAM3::System* pSLAM, ros::Publisher pub, ros::Publisher reward_pub, ros::Publisher episode_pub) {
            this->mpSLAM = pSLAM;
            this->pub = pub;
            this->reward_pub = reward_pub;
            this->episode_pub = episode_pub;
        }

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void RespondAgent(const std_msgs::String msg);

    ORB_SLAM3::System* mpSLAM;
};
    





int main(int argc, char **argv)
{
    std::cout << "I copied\n";
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);
    ros::NodeHandle nodeHandler;

    ros::Publisher pub = nodeHandler.advertise<sensor_msgs::Image>("ORB_SLAM3/State",10,true);
    ros::Publisher reward_pub = nodeHandler.advertise<std_msgs::Float32>("ORB_SLAM3/Reward",10,true);
    ros::Publisher state_pub = nodeHandler.advertise<std_msgs::String>("ORB_SLAM3/Episode",10,true);

    ImageGrabber igb(&SLAM, pub, reward_pub, state_pub);

    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber agentsub = nodeHandler.subscribe("/ORB_SLAM3/AGENT_SLAM", 1, &ImageGrabber::RespondAgent, &igb);


    std::cout << "I saved\n";
    
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::RespondAgent(const std_msgs::String msg){
    mpSLAM->GetTracker()->Reset(1);
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
        if(cv_ptr != 0)
    {
        std::cout << "This happened\n";
    } 
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    mpSLAM->GetFrameDrawer()->PublishState();
    cv::Mat stateImg = mpSLAM->GetFrameDrawer()->orbslam_state;
    cout << "starting calculating reward" << endl; 
    mpSLAM->GetFrameDrawer()->CalculateReward();
    cout << "starting calculating episode" << endl;
    mpSLAM->GetFrameDrawer()->PublishEpisode();

    cout << "calculated reward" << endl;

    float reward = mpSLAM->GetFrameDrawer()->reward;
    std_msgs::Float32 reward_msg;
    reward_msg.data = reward;

    std::string episode = mpSLAM->GetFrameDrawer()->episodeState;
    std_msgs::String episode_msg;
    episode_msg.data = episode;

    cout << "got episode information";
    
    cv_bridge::CvImage rosImage;
    rosImage.image = stateImg.clone();
    rosImage.header.stamp = ros::Time::now();
    rosImage.encoding = "bgr8";

    pub.publish(rosImage.toImageMsg());
    reward_pub.publish(reward_msg);
    episode_pub.publish(episode_msg);
}

