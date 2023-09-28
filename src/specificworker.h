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
#define DEBUG 0

#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Node.hpp>
#include <webots/Field.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define TIME_STEP 64
// Robot geometry
#define WHEEL_RADIUS 0.08
#define LX 0.135  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.237  // lateral distance from robot's COM to wheel [m].
// Human control
#define NUMBER_OF_HUMANS_IN_SCENE 1

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

	RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
	RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance);

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);

	void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();

	RoboCompLaserMulti::LaserConfData LaserMulti_getLaserConfData(int robotid);
	RoboCompLaserMulti::TLaserData LaserMulti_getLaserData(int robotid);

	void OmniRobotMulti_getBasePose(int robotId, int &x, int &z, float &alpha);
	void OmniRobotMulti_setSpeedBase(int robotId, float advx, float advz, float rot);
	void OmniRobotMulti_stopBase(int robotId);

	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

public slots:
	void compute();
	int startup_check();

	void initialize(int period);
private:
	bool startup_check_flag;

    webots::Supervisor* robot;
    webots::Lidar* lidar_helios;
    webots::Lidar* lidar_pearl;
    webots::Camera* camera;
    webots::RangeFinder* range_finder;
    webots::Camera* camera360_1;
    webots::Camera* camera360_2;
    webots::Motor *motors[4];
    webots::PositionSensor *ps[4];
    webots::Node* humans[NUMBER_OF_HUMANS_IN_SCENE];

    void receiving_lidarData(webots::Lidar* _lidar, RoboCompLidar3D::TData &_lidar3dData, const Eigen::Affine3f &_extrinsic_matix = Eigen::Affine3f::Identity());
    void receiving_cameraRGBData(webots::Camera* _camera);
    void receiving_depthImageData(webots::RangeFinder* _rangeFinder);
    void receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2);

    RoboCompLidar3D::TData filterLidarData(const RoboCompLidar3D::TData _lidar3dData, float _start, float _len, int _decimationDegreeFactor);

    // Laser
    RoboCompLaser::TLaserData laserData;
    RoboCompLaser::LaserConfData laserDataConf;

    // Lidar3d
    RoboCompLidar3D::TData lidar3dData_helios;
    RoboCompLidar3D::TData lidar3dData_pearl;

    // Camera RGBD simple
    RoboCompCameraRGBDSimple::TDepth depthImage;
    RoboCompCameraRGBDSimple::TImage cameraImage;

    // Camera 360
    RoboCompCamera360RGB::TImage camera360Image;

    // Auxiliar functions
    void printNotImplementedWarningMessage(string functionName);

	//Extrinsic
	Eigen::Affine3f extrinsic_helios, extrinsic_bpearl;
	Eigen::Vector3f box_min;
	Eigen::Vector3f box_max;
	float floor_line;
	inline bool isPointOutsideCube(const Eigen::Vector3f point, const Eigen::Vector3f box_min, const Eigen::Vector3f box_max);
};

#endif
