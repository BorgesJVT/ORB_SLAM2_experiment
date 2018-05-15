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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include"Map.h"
#include"MapPoint.h"
#include"KeyFrame.h"
#include<pangolin/pangolin.h>
#include<opencv2/core/core.hpp>
#include<ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include<mutex>

namespace ORB_SLAM2
{
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;

class MapDrawer
{
public:
    MapDrawer(Map* pMap, const string &strSettingPath);

    Map* mpMap;

    void DrawMapPoints();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;

	ros::NodeHandle nh;
    ros::Publisher publisher;
	ros::Publisher pub_blackpoints;
	ros::Publisher pub_redpoints;
	ros::Publisher pub_move_camera;

    float fCameraSize;
	tf2_ros::TransformBroadcaster tfb;

	geometry_msgs::TransformStamped baseMovingCamera;
    visualization_msgs::Marker mCurrentCamera;
	visualization_msgs::Marker mAllCamera;

	PointCloud::Ptr msg_black;
	PointCloud2::Ptr msg_black2;
	PointCloud::Ptr msg_red;
	PointCloud2::Ptr msg_red2;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
