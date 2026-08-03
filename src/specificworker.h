/*
 *    Copyright (C) 2025 by YOUR NAME HERE
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

// If you want to reduce the period automatically due to lack of use, you must uncomment the following line
#define HIBERNATION_ENABLED

#include <genericworker.h>
#include <webots/Robot.hpp>
#include <webots/Lidar.hpp>
#include <webots/Camera.hpp>
#include <webots/RangeFinder.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <doublebuffer/DoubleBuffer.h>
#include <fps/fps.h>
#include "fixedsizedeque.h"
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <map>
#include <string>
#include <vector>

using namespace Eigen;
using namespace std;
#define TIME_STEP 33
// robot geometry
#define WHEEL_RADIUS 0.05
#define LX 0.135  // longitudinal distance from robot's COM to wheel [m].
#define LY 0.237  // lateral distance from robot's COM to wheel [m].
#define LINEAR_MAX_SPEED 1.5 // meters per second 
#define ANGULAR_MAX_SPEED 4.03 //radians per second

/**
 * \brief Class SpecificWorker implements the core functionality of the component.
 */
class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
    /**
     * \brief Constructor for SpecificWorker.
     * \param configLoader Configuration loader for the component.
     * \param tprx Tuple of proxies required for the component.
     * \param startup_check Indicates whether to perform startup checks.
     */
	SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check);

	/**
     * \brief Destructor for SpecificWorker.
     */
	~SpecificWorker();

	RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);
	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera);

	RoboCompIMU::Acceleration IMU_getAcceleration();
	RoboCompIMU::Gyroscope IMU_getAngularVel();
	RoboCompIMU::DataImu IMU_getDataImu();
	RoboCompIMU::Magnetic IMU_getMagneticFields();
	RoboCompIMU::Orientation IMU_getOrientation();
	void IMU_resetImu();

	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();

	RoboCompLidar3D::TColorCloudData Lidar3D_getColorCloudData();
	RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor);
	RoboCompLidar3D::TDataImage Lidar3D_getLidarDataArrayProyectedInImage(std::string name);
	RoboCompLidar3D::TDataCategory Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp);
	RoboCompLidar3D::TData Lidar3D_getLidarDataProyectedInImage(std::string name);
	RoboCompLidar3D::TData Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor);


	void OmniRobot_correctOdometer(int x, int z, float alpha);
	void OmniRobot_getBasePose(int &x, int &z, float &alpha);
	void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void OmniRobot_resetOdometer();
	void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void OmniRobot_setOdometerPose(int x, int z, float alpha);
	void OmniRobot_setSpeedBase(float advx, float advz, float rot);
	void OmniRobot_stopBase();

	RoboCompVisualElements::TObjects VisualElements_getVisualObjects(RoboCompVisualElements::TObjects objects);
	void VisualElements_setVisualObjects(RoboCompVisualElements::TObjects objects);

	RoboCompWebots2Robocomp::ObjectPose Webots2Robocomp_getObjectPose(std::string DEF);
	void Webots2Robocomp_resetWebots();
	void Webots2Robocomp_setDoorAngle(float angle);
	void Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path);
    void Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles){};
	void Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose){};
	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

public slots:

	/**
	 * \brief Initializes the worker one time.
	 */
	void initialize();

	/**
	 * \brief Main compute loop of the worker.
	 */
	void compute();

	/**
	 * \brief Handles the emergency state loop.
	 */
	void emergency();

	/**
	 * \brief Restores the component from an emergency state.
	 */
	void restore();

    /**
     * \brief Performs startup checks for the component.
     * \return An integer representing the result of the checks.
     */
	int startup_check();
private:

	/**
     * \brief Flag indicating whether startup checks are enabled.
     */
	bool startup_check_flag;

    FPSCounter fps;
    std::atomic<std::chrono::high_resolution_clock::time_point> last_read;
    int MAX_INACTIVE_TIME = 5;  // secs after which the component is paused. It reactivates with a new reset

    webots::Node* robotNode;
    webots::Supervisor* robot;
    webots::Lidar* lidar_helios;
    webots::Lidar* lidar_pearl;
    webots::Camera* camera;
    webots::RangeFinder* range_finder;
    webots::Camera* camera360_1;
    webots::Camera* camera360_2;
    webots::Motor *motors[4];
    webots::PositionSensor *ps[4];
    webots::Accelerometer* accelerometer;
    webots::Gyro* gyroscope;
	webots::Camera* zed;
	webots::RangeFinder* zedRangeFinder;
	webots::Node* controllableDoor;
    webots::Field* robotRotationField = nullptr;
    webots::Field* robotTranslationField = nullptr;

    // --- OmniRobot RPC decoupling ------------------------------------------------
    // The Webots libController API must only be touched from the thread that calls
    // robot->step(). Supervisor getters block until the simulator answers, so an Ice
    // servant thread calling them lands anywhere inside the current step and the RPC
    // latency swings between ~0 and one full period. Servants therefore only touch
    // these snapshots; compute() is the single point that talks to Webots.
    struct SpeedCommand { float advx = 0.f, advz = 0.f, rot = 0.f; };
    std::mutex speed_command_mutex;
    SpeedCommand speed_command;
    bool speed_command_pending = false;

    std::mutex base_state_mutex;
    RoboCompGenericBase::TBaseState base_state{};

    void apply_pending_speed_command();
    void update_base_state(const double *position, const Eigen::Vector2f &velocity_local, double rot_velocity);

    // --- Deferred Webots work ----------------------------------------------------
    // Servants that need to *write* to the simulation queue a closure here; compute()
    // drains the queue in the Webots thread just before stepping.
    std::mutex webots_tasks_mutex;
    std::vector<std::function<void()>> webots_tasks;
    void post_webots_task(std::function<void()> task);
    void run_webots_tasks();

    // --- IMU snapshot ------------------------------------------------------------
    std::mutex imu_mutex;
    RoboCompIMU::DataImu imu_data{};
    void update_imu_data(long timestamp);

    // --- VisualElements snapshot -------------------------------------------------
    // Also removes the race on humanObjects, which compute() rewrites in
    // parseHumanObjects() while a servant thread was iterating it.
    std::mutex visual_objects_mutex;
    RoboCompVisualElements::TObjects visual_objects;
    void update_visual_objects();

    // --- Object pose cache -------------------------------------------------------
    // getObjectPose() takes an arbitrary DEF, so it cannot be snapshotted up front.
    // Instead each DEF is registered on first use and refreshed every cycle from then
    // on: only the very first call for a given DEF waits for the Webots thread.
    struct TrackedObject
    {
        RoboCompWebots2Robocomp::ObjectPose pose{};
        bool resolved = false;      // compute() has looked this DEF up at least once
    };
    struct TrackedObjectHandles     // compute()-thread only, never locked
    {
        webots::Field *translation = nullptr;
        webots::Field *rotation = nullptr;
    };
    std::mutex object_poses_mutex;
    std::condition_variable object_poses_cv;
    std::map<std::string, TrackedObject> object_poses;
    std::map<std::string, TrackedObjectHandles> object_handles;
    void update_object_poses();

    void receiving_lidarData(std::string name, webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData>& lidar_doubleBuffer, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp);
    void receiving_cameraRGBData(webots::Camera* _camera, long timestamp);
    //void receiving_depthImageData(webots::RangeFinder* _rangeFinder, long timestamp);
    void receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp);
    void receiving_robotSpeed(webots::Supervisor* _robot, long timestamp);
	void receiving_cameraRGBD(webots::Camera* _camera, webots::RangeFinder* _rangeFinder, RoboCompCameraRGBDSimple::TRGBD& _image, long timestamp);
    double generate_noise(double stddev);

    // Laser
    RoboCompLaser::TLaserData laserData;
    RoboCompLaser::LaserConfData laserDataConf;

    // Camera RGBD simple
    RoboCompCameraRGBDSimple::TDepth depthImage;
    RoboCompCameraRGBDSimple::TImage cameraImage;

    // Camera 360
    RoboCompCamera360RGB::TImage camera360Image;

    // Human Tracking
    struct WebotsHuman{
        webots::Node *node;
        RoboCompGridder::TPath path;
        RoboCompGridder::TPoint currentTarget;
    };

    std::map<int, WebotsHuman> humanObjects;
    void parseHumanObjects();

    // Auxiliar functions
    void printNotImplementedWarningMessage(std::string functionName);

    // Webots2RoboComp interface
    void moveHumanToNextTarget(int humanId);
    void humansMovement();
    void set_path_to_human(int humanId, const RoboCompGridder::TPath &path);

    // Sampling period (ms) for the heavy sensors: lidars, 360 camera, zed. Decoupled
    // from Period.Compute so the control/actuation loop can step fast without
    // reprocessing point clouds and images at the same rate.
    int sensor_period = 0;
    double next_sensor_sample_ms = 0.0;   // next simulation time a sample is due

    struct PARAMS
    {
        bool delay = false;
        bool do_joystick = true;
        bool points3D = false;
        bool camera360 = false;
        bool humans = false;
    };
    PARAMS pars;

    FixedSizeDeque<RoboCompCamera360RGB::TImage> camera_queue{10};
    //Is it necessary to use two lidar queues? One for each lidaR?
    FixedSizeDeque<RoboCompLidar3D::TData> pearl_delay_queue{10};
    FixedSizeDeque<RoboCompLidar3D::TData> helios_delay_queue{10};

    // Double buffer
    DoubleBuffer<RoboCompCamera360RGB::TImage, RoboCompCamera360RGB::TImage> double_buffer_360;


    //Lidar3D doublebuffer
    DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> double_buffer_helios;
    DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> double_buffer_pearl;

	// zed
	RoboCompCameraRGBDSimple::TRGBD zedImage;
	DoubleBuffer<RoboCompCameraRGBDSimple::TRGBD, RoboCompCameraRGBDSimple::TRGBD> double_buffer_zed;

    Matrix4d create_affine_matrix(double a, double b, double c, Vector3d trans);
    std::tuple<float, float, float> rotationMatrixToEulerZYX(const double* R);

	void setDoorAperture(float _aperture);

signals:
        //void customSignal();
};

#endif
