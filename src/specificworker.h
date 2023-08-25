/*
 *    Copyright (C) 2023 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
	\brief
	@author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);

	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();

	RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, int start, int len, int decimationfactor);

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);


public slots:
	void compute();
	int startup_check();

	void initialize(int period);
private:
	bool startup_check_flag;

    webots::Robot* robot;
    webots::Lidar* lidar;
    webots::Camera* camera;
    webots::RangeFinder* range_finder;
    webots::Camera* camera360_1;
    webots::Camera* camera360_2;



    void receiving_lidarData(webots::Lidar* _lidar);
    void receiving_cameraRGBData(webots::Camera* _camera);
    void receiving_depthImageData(webots::RangeFinder* _rangeFinder);
    void receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2);

    // Laser
    RoboCompLaser::TLaserData laserData;
    RoboCompLaser::LaserConfData laserDataConf;

    // Lidar3d
    RoboCompLidar3D::TData lidar3dData;

    // Camera RGBD simple
    RoboCompCameraRGBDSimple::TDepth depthImage;
    RoboCompCameraRGBDSimple::TImage cameraImage;

    // Camera 360
    RoboCompCamera360RGB::TImage camera360Image;

    // Auxiliar functions
    void printNotImplementedWarningMessage(string functionName);
};

#endif
