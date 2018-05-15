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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


namespace ORB_SLAM2
{
//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

	/* ROS part of constructor */
	//const char* MAP_FRAME_ID = "/ORB_SLAM2/World";
    const char* MAP_FRAME_ID = "base_camera";
    const char* CAMERA_NAMESPACE = "Camera";
    fCameraSize=0.04;
    mCurrentCamera.header.frame_id = MAP_FRAME_ID;
    mCurrentCamera.ns = CAMERA_NAMESPACE;
    mCurrentCamera.id=4;
    mCurrentCamera.type = visualization_msgs::Marker::LINE_LIST;
    mCurrentCamera.scale.x=0.001;//0.2; 0.03
    mCurrentCamera.pose.orientation.w=1.0;
    mCurrentCamera.action=visualization_msgs::Marker::ADD;
    mCurrentCamera.color.g=1.0f;
    mCurrentCamera.color.a = 1.0;

	mAllCamera.header.frame_id = MAP_FRAME_ID;
    mAllCamera.ns = CAMERA_NAMESPACE;
    mAllCamera.id=4;
    mAllCamera.type = visualization_msgs::Marker::LINE_LIST;
    mAllCamera.scale.x=0.001;//0.2; 0.03
    mAllCamera.pose.orientation.w=1.0;
    mAllCamera.action=visualization_msgs::Marker::ADD;
    mAllCamera.color.b=1.0f;
    mAllCamera.color.a = 1.0;

	baseMovingCamera.header.frame_id = "base_camera";
  	baseMovingCamera.child_frame_id = "moving_camera";
	pub_move_camera = nh.advertise<geometry_msgs::TransformStamped>("/ORB_SLAM2/base_moving_link", 10);
	publisher = nh.advertise<visualization_msgs::Marker>("/ORB_SLAM2/CameraMarker", 10);
	/* Black Points */
	PointCloud::Ptr black (new PointCloud);
    PointCloud2::Ptr black2 (new PointCloud2);
	this->msg_black = black;
	this->msg_black2 = black2;
	msg_black->header.frame_id = "base_camera";
	pub_blackpoints = nh.advertise<PointCloud2> ("ORB_SLAM2/black_points", 1000);
	/* Red Points */
	PointCloud::Ptr red (new PointCloud);
    PointCloud2::Ptr red2 (new PointCloud2);
	this->msg_red = red;
	this->msg_red2 = red2;
	msg_red->header.frame_id = "base_camera";
	pub_redpoints = nh.advertise<PointCloud2> ("ORB_SLAM2/red_points", 1000, true);
	
}

void MapDrawer::DrawMapPoints()
{
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());	

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

	//ROS point cloud config
	msg_black->points.clear();
	msg_black->width = vpMPs.size();
    msg_black->height = 1;
	msg_black->points.resize(msg_black->width * msg_black->height);

	msg_red->points.clear();
	msg_red->width = spRefMPs.size();
    msg_red->height = 1;
	msg_red->points.resize(msg_red->width * msg_red->height);
	
    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
		//std::cout << "x: " << pos.at<float>(0) << " y: " << pos.at<float>(1) << " z: " << pos.at<float>(2) << std::endl;
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
		//msg_black->points.push_back(pcl::PointXYZ(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)));
		msg_black->points[i].x = pos.at<float>(0);
        msg_black->points[i].y = pos.at<float>(1);
        msg_black->points[i].z = pos.at<float>(2);
		//msg_black->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        //msg_black->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        //msg_black->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

	size_t i = 0;
	uint8_t r(255), g(15), b(15);
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
		//msg->points.push_back(pcl::PointXYZ(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2)));
		msg_red->points[i].x = pos.at<float>(0);
        msg_red->points[i].y = pos.at<float>(1);
        msg_red->points[i].z = pos.at<float>(2);
		msg_red->points[i].rgb = *reinterpret_cast<float*>(&rgb);
		i++;
    }
    glEnd();
	//std::cout << "#############################" << std::endl;
	//ROS publishing points
	/*for (size_t i = 0; i < msg->points.size (); ++i) {
      msg->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      msg->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      msg->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }*/
	toPCLPointCloud2 (*msg_black, *msg_black2);
	msg_black2->is_bigendian = true;
	pub_blackpoints.publish(msg_black2);

	toPCLPointCloud2 (*msg_red, *msg_red2);
	msg_red2->is_bigendian = true;
	pub_redpoints.publish(msg_red2);
}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;
    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();
/*
		std::cout << Twc.at<float>(0,0) << " " << Twc.at<float>(0,1) << " " << Twc.at<float>(0,2) << " " << Twc.at<float>(0,3) << std::endl;
		std::cout << Twc.at<float>(1,0) << " " << Twc.at<float>(1,1) << " " << Twc.at<float>(1,2) << " " << Twc.at<float>(1,3) << std::endl;
		std::cout << Twc.at<float>(2,0) << " " << Twc.at<float>(2,1) << " " << Twc.at<float>(2,2) << " " << Twc.at<float>(2,3) << std::endl;
		std::cout << Twc.at<float>(3,0) << " " << Twc.at<float>(3,1) << " " << Twc.at<float>(3,2) << " " << Twc.at<float>(3,3) << std::endl;
*/
            glPushMatrix();
            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth);
            glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();

			/* ROS part of drawing key frames*/
			/*
			cv::Mat o = (cv::Mat_<float>(4,1) <<   0,  0, 0, 1);
   			cv::Mat p1 = (cv::Mat_<float>(4,1) <<  w,  h, z, 1);
   			cv::Mat p2 = (cv::Mat_<float>(4,1) <<  w, -h, z, 1);
   			cv::Mat p3 = (cv::Mat_<float>(4,1) << -w, -h, z, 1);
		    cv::Mat p4 = (cv::Mat_<float>(4,1) << -w,  h, z, 1);
			cv::Mat ow =  Twc*o;
   			cv::Mat p1w = Twc*p1;
		    cv::Mat p2w = Twc*p2;
   			cv::Mat p3w = Twc*p3;
   			cv::Mat p4w = Twc*p4;
			geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
		    msgs_o.x=ow.at<float>(0);
		    msgs_o.y=ow.at<float>(1);
		    msgs_o.z=ow.at<float>(2);
		    msgs_p1.x=p1w.at<float>(0);
		    msgs_p1.y=p1w.at<float>(1);
		    msgs_p1.z=p1w.at<float>(2);
		    msgs_p2.x=p2w.at<float>(0);
		    msgs_p2.y=p2w.at<float>(1);
		    msgs_p2.z=p2w.at<float>(2);
		    msgs_p3.x=p3w.at<float>(0);
		    msgs_p3.y=p3w.at<float>(1);
		    msgs_p3.z=p3w.at<float>(2);
		    msgs_p4.x=p4w.at<float>(0);
		    msgs_p4.y=p4w.at<float>(1);
		    msgs_p4.z=p4w.at<float>(2);

		    mAllCamera.points.clear();		   
		    mAllCamera.points.push_back(msgs_o);
		    mAllCamera.points.push_back(msgs_p1);
		    mAllCamera.points.push_back(msgs_o);
		    mAllCamera.points.push_back(msgs_p2);
		    mAllCamera.points.push_back(msgs_o);
		    mAllCamera.points.push_back(msgs_p3);
		    mAllCamera.points.push_back(msgs_o);
		    mAllCamera.points.push_back(msgs_p4);
		    mAllCamera.points.push_back(msgs_p1);
		    mAllCamera.points.push_back(msgs_p2);
		    mAllCamera.points.push_back(msgs_p2);
		    mAllCamera.points.push_back(msgs_p3);
		    mAllCamera.points.push_back(msgs_p3);
		    mAllCamera.points.push_back(msgs_p4);
		    mAllCamera.points.push_back(msgs_p4);
		    mAllCamera.points.push_back(msgs_p1);
		   
		    mAllCamera.header.stamp = ros::Time::now();
		   
		    publisher.publish(mAllCamera);
			*/
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();

	/* ROS Camera Publisher */
	mCurrentCamera.points.clear();	
	if(mCameraPose.empty()) return;

	//Camera is a pyramid. Define in camera coordinate system
    cv::Mat o = (cv::Mat_<float>(4,1) <<   0,  0, 0, 1);
    cv::Mat p1 = (cv::Mat_<float>(4,1) <<  w,  h, z, 1);
    cv::Mat p2 = (cv::Mat_<float>(4,1) <<  w, -h, z, 1);
    cv::Mat p3 = (cv::Mat_<float>(4,1) << -w, -h, z, 1);
    cv::Mat p4 = (cv::Mat_<float>(4,1) << -w,  h, z, 1);

	cv::Mat poseInv = mCameraPose.inv();
/*
		std::cout << poseInv.at<float>(0,0) << " " << poseInv.at<float>(0,1) << " " << poseInv.at<float>(0,2) << " " << poseInv.at<float>(0,3) << std::endl;
		std::cout << poseInv.at<float>(1,0) << " " << poseInv.at<float>(1,1) << " " << poseInv.at<float>(1,2) << " " << mCameraPose.at<float>(1,3) << std::endl;
		std::cout << poseInv.at<float>(2,0) << " " << poseInv.at<float>(2,1) << " " << poseInv.at<float>(2,2) << " " << poseInv.at<float>(2,3) << std::endl;
		std::cout << poseInv.at<float>(3,0) << " " << poseInv.at<float>(3,1) << " " << poseInv.at<float>(3,2) << " " << poseInv.at<float>(3,3) << std::endl;
*/

	cv::Mat ow =  poseInv*o;
    cv::Mat p1w = poseInv*p1;
    cv::Mat p2w = poseInv*p2;
    cv::Mat p3w = poseInv*p3;
    cv::Mat p4w = poseInv*p4;

	baseMovingCamera.transform.translation.x = poseInv.at<float>(0, 3);
  	baseMovingCamera.transform.translation.y = poseInv.at<float>(1, 3);
  	baseMovingCamera.transform.translation.z = poseInv.at<float>(2, 3);

	tf::Matrix3x3 tf3d;
	tf3d.setValue(poseInv.at<float>(0, 0), poseInv.at<float>(0, 1), poseInv.at<float>(0, 2),
				  poseInv.at<float>(1, 0), poseInv.at<float>(1, 1), poseInv.at<float>(1, 2),
				  poseInv.at<float>(2, 0), poseInv.at<float>(2, 1), poseInv.at<float>(2, 2));
  	tf2::Quaternion q;
	double row, pitch, yaw;
	tf3d.getRPY(row, pitch, yaw);
	q.setRPY(row, pitch, yaw);
  	baseMovingCamera.transform.rotation.x = q.x();
  	baseMovingCamera.transform.rotation.y = q.y();
  	baseMovingCamera.transform.rotation.z = q.z();
  	baseMovingCamera.transform.rotation.w = q.w();

	//baseMovingCamera.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
    //baseMovingCamera.transform.translation.y = 2.0*cos(ros::Time::now().toSec());
    tfb.sendTransform(baseMovingCamera);

	geometry_msgs::Point msgs_o,msgs_p1, msgs_p2, msgs_p3, msgs_p4;
    msgs_o.x=ow.at<float>(0);
    msgs_o.y=ow.at<float>(1);
    msgs_o.z=ow.at<float>(2);
    msgs_p1.x=p1w.at<float>(0);
    msgs_p1.y=p1w.at<float>(1);
    msgs_p1.z=p1w.at<float>(2);
    msgs_p2.x=p2w.at<float>(0);
    msgs_p2.y=p2w.at<float>(1);
    msgs_p2.z=p2w.at<float>(2);
    msgs_p3.x=p3w.at<float>(0);
    msgs_p3.y=p3w.at<float>(1);
    msgs_p3.z=p3w.at<float>(2);
    msgs_p4.x=p4w.at<float>(0);
    msgs_p4.y=p4w.at<float>(1);
    msgs_p4.z=p4w.at<float>(2);

	mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_o);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p2);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p3);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p4);
    mCurrentCamera.points.push_back(msgs_p1);

	mCurrentCamera.header.stamp = ros::Time::now();
    
    publisher.publish(mCurrentCamera);
	
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
		/*
		std::cout << "#################################" << std::endl;
		std::cout << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1) << " " << Rwc.at<float>(0,2) << std::endl;
		std::cout << Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1) << " " << Rwc.at<float>(1,2) << std::endl;
		std::cout << Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1) << " " << Rwc.at<float>(2,2) << std::endl;
		*/
		//std::cout << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << std::endl;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM
