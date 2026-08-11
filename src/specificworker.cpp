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
#include "specificworker.h"

#pragma region Robocomp Methods

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}

SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;

    if(robot)
        delete robot;
}

void SpecificWorker::initialize()
{
    std::cout << "Initialize worker" << std::endl;
    if(this->startup_check_flag)
    {
        this->startup_check();
    }
    else
    {
         try { pars.points3D = configLoader.get<bool>("Camera.ZED.points3D"); } catch(...) {}
         try { pars.humans = configLoader.get<bool>("Humans"); } catch(...) {}

        // Which doors this provider advertises over DoorControl: the hinge DEFs of the loaded world.
        // Absent from the config it advertises none, and getCapabilities() reports canActuate=false —
        // which is the honest answer for a world whose doors are scenery, and is what stops a caller
        // offering an open-the-door affordance it could never carry out.
        try { door_control_defs = configLoader.get<std::vector<std::string>>("DoorControl.Doors"); }
        catch(...) {}
        if (not door_control_defs.empty())
            std::cout << "[DoorControl] advertising " << door_control_defs.size() << " door(s)\n";

        // Heavy sensors sample independently of the step rate; absent from the config
        // they follow it, which is the historical behaviour.
        const int compute_period = this->getPeriod("Compute");
        sensor_period = compute_period;
        try { sensor_period = configLoader.get<int>("Period.Sensors"); } catch(...) {}
        if(sensor_period < compute_period)
        {
            std::cout << "Period.Sensors (" << sensor_period << ") is below Period.Compute ("
                      << compute_period << "); clamping to Period.Compute." << std::endl;
            sensor_period = compute_period;
        }
        std::cout << "Compute period: " << compute_period << " ms, sensor period: "
                  << sensor_period << " ms" << std::endl;

        // pause variable
        last_read.store(std::chrono::high_resolution_clock::now());

        robot = new webots::Supervisor();
        robotNode = robot->getFromDef("shadow");
        // Resolved once: getField() is a name lookup against the simulator.
        if(robotNode)
        {
            robotRotationField = robotNode->getField("rotation");
            robotTranslationField = robotNode->getField("translation");
        }

        // Inicializa los motores y los sensores de posición.
        const char *motorNames[4] = {"wheel2", "wheel1", "wheel4", "wheel3"};
        //const char *sensorNames[4] = {"wheel1sensor", "wheel2sensor", "wheel3sensor", "wheel4sensor"};

        // Inicializa los sensores soportados.
        lidar_helios = robot->getLidar("helios");
        lidar_pearl = robot->getLidar("bpearl");
        camera = robot->getCamera("camera");
        range_finder = robot->getRangeFinder("range-finder");
        camera360_1 = robot->getCamera("camera_360_1");
        camera360_2 = robot->getCamera("camera_360_2");
        accelerometer = robot->getAccelerometer("accelerometer");
        gyroscope = robot->getGyro("gyro");
        zedRangeFinder = robot->getRangeFinder("zed-ranger");
        zed = robot->getCamera("zed");

        // Activa los componentes en la simulación si los detecta.
        // Heavy sensors run at sensor_period; the IMU and wheel encoders are cheap and
        // feed the control loop, so they stay at the step rate.
        if(lidar_helios) lidar_helios->enable(sensor_period);
        if(lidar_pearl) lidar_pearl->enable(sensor_period);
        if(camera) camera->enable(sensor_period);
        if(range_finder) range_finder->enable(sensor_period);
        if(camera360_1 && camera360_2){
            camera360_1->enable(sensor_period);
            camera360_2->enable(sensor_period);
        }
        for (int i = 0; i < 4; i++)
        {
            motors[i] = robot->getMotor(motorNames[i]);
            ps[i] = motors[i]->getPositionSensor();
            ps[i]->enable(compute_period);
            motors[i]->setPosition(INFINITY); // Modo de velocidad.
            motors[i]->setVelocity(0);
        }
        if(accelerometer) accelerometer->enable(compute_period);
        if(gyroscope) gyroscope->enable(compute_period);
        if (zedRangeFinder) zedRangeFinder->enable(sensor_period);
        if (zed) zed->enable(sensor_period);

        // Doors are resolved lazily, by DEF, on the first setDoorAngle that names them (see
        // setDoorAperture) — there is no single "the" controllable door any more.
    }
}

void SpecificWorker::compute()
{
    // Getting simulation timestamp
    //double now = robot->getTime() * 1000;
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    // Webots only refreshes a device's data once per its sampling period, so when the
    // step rate is faster than sensor_period the extra passes would reprocess the same
    // point clouds and images. Skip them.
    const double sim_now_ms = robot != nullptr ? robot->getTime() * 1000.0 : 0.0;
    const bool new_sensor_sample = sim_now_ms >= next_sensor_sample_ms;
    if(new_sensor_sample)
        next_sensor_sample_ms = sim_now_ms + sensor_period;

    // Getting the data from simulation. The robot pose/speed feeds the control loop,
    // so it is refreshed every step regardless of the sensor rate.
    if(robot) receiving_robotSpeed(robot, now);
    if(new_sensor_sample)
    {
        if(lidar_helios) receiving_lidarData("helios", lidar_helios, double_buffer_helios,  helios_delay_queue, now);
        if(lidar_pearl) receiving_lidarData("bpearl", lidar_pearl, double_buffer_pearl, pearl_delay_queue, now);
        //if(camera) receiving_cameraRGBData(camera, now);
        //if(range_finder) receiving_depthImageData(range_finder, now);
        if(camera360_1 and camera360_2) receiving_camera360Data(camera360_1, camera360_2, now);
        if(zedRangeFinder && zed) receiving_cameraRGBD(zed, zedRangeFinder, zedImage, now);
    }

    // Refresh the snapshots served by the RPC servants, then drain any work they
    // queued. Everything that talks to Webots happens on this thread.
    update_imu_data(now);
    update_object_poses();
    update_door_control();
    run_webots_tasks();

    // Push any speed command received over RPC since the last step.
    apply_pending_speed_command();

    robot->step(getPeriod("Compute"));

//    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;

    if(pars.humans)
    {
        parseHumanObjects();
        update_visual_objects();
    }

    //humansMovement();
    fps.print("FPS:");
}

void SpecificWorker::emergency()
{
    std::cout << "Emergency worker" << std::endl;
	//computeCODE
	//
	//if (SUCCESSFUL)
    //  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
    std::cout << "Restore worker" << std::endl;
	//computeCODE
	//Restore emergency component

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}

#pragma endregion Robocomp Methods

#pragma region Data-Catching Methods

void SpecificWorker::receiving_cameraRGBD(webots::Camera* _camera,
                                          webots::RangeFinder* _rangeFinder,
                                          RoboCompCameraRGBDSimple::TRGBD& _image,
                                          long timestamp)
{

    RoboCompCameraRGBDSimple::TRGBD new_zed_image;

    // ------------------ Common -----------------------------
    new_zed_image.image.alivetime = new_zed_image.depth.alivetime = new_zed_image.points.alivetime = timestamp;
    new_zed_image.image.period = new_zed_image.depth.period = new_zed_image.points.period = fps.get_period();

    int width = _camera->getWidth();
    int height = _camera->getHeight();
    double cfov = _camera->getFov(); // radianes
    double cfx = width / (2.0 * tan(cfov / 2.0));
    double cfy = cfx; // square pixel assumption //height / (2.0 * tan(cfov / 2.0));
    new_zed_image.image.focalx = cfx;    //TODO: cambiar el tipo en el IDSL
    new_zed_image.image.focaly = cfy;
    new_zed_image.image.width = new_zed_image.depth.width = width;
    new_zed_image.image.height = new_zed_image.depth.height = height;
    new_zed_image.image.depth = 3; // RGB
    new_zed_image.image.compressed = new_zed_image.depth.compressed = new_zed_image.points.compressed = false;

    // -------------------- Imagen RGB --------------------
    const unsigned char *webotsImageData = _camera->getImage();
    if (webotsImageData == nullptr) return;   // no sample yet (first cycle, before any step)
    cv::Mat imageMatBGRA(height, width, CV_8UC4, (void*)webotsImageData);

    cv::Mat imageMatRGB;
    cv::cvtColor(imageMatBGRA, imageMatRGB, cv::COLOR_BGRA2RGB);

    new_zed_image.image.image.resize(imageMatRGB.total() * imageMatRGB.channels());
    new_zed_image.image.depth = imageMatRGB.channels();

    std::memcpy(new_zed_image.image.image.data(), imageMatRGB.data, new_zed_image.image.image.size());
    // TODO: check if this can be done with move_iterator

    // -------------------- Imagen de profundidad --------------------
    double fov = _rangeFinder->getFov(); // radianes
    double fx = width / (2.0 * tan(fov / 2.0));
    double fy = fx;
    new_zed_image.depth.focalx = fx;    //TODO: cambiar el tipo en el IDSL
    new_zed_image.depth.focaly = fy;
    new_zed_image.depth.width = _rangeFinder->getWidth();
    new_zed_image.depth.height = _rangeFinder->getHeight();
    new_zed_image.depth.depthFactor = 1; // meters

    const float* depthImage = _rangeFinder->getRangeImage();
    cv::Mat depthMat(height, width, CV_32FC1, (void*)depthImage);
    new_zed_image.depth.depth.resize(width * height * sizeof(float));
    // check conditions to avoid problems
    if (depthMat.data == nullptr)
    {
        qWarning() << "Error: Depth data is null.";
        return;
    }
    std::memcpy(new_zed_image.depth.depth.data(), depthMat.data, new_zed_image.depth.depth.size());

    // -------------------- Point Cloud --------------------
    if(pars.points3D)
    {
        RoboCompCameraRGBDSimple::TPoints& points = new_zed_image.points;
        points.alivetime = timestamp;
        points.period = fps.get_period();
        points.compressed = false;
        int offset = 2;
        points.points.reserve((width / offset) * (height / offset));
        for (int v = 0; v < height; v=v+offset)
            for (int u = 0; u < width; u=u+offset)
            {
                float Z = depthMat.at<float>(v, u);
                // if Z is nan or zero, skip the point
                if (std::isnan(Z) || Z <= 0.0f || Z > 10000.0f) continue;
                float cx = width / 2.0f;
                float cy = height / 2.0f;
                float X = (u - cx) * Z / fx;
                float Y = (v - cy) * Z / fy;
                points.points.emplace_back(X, Y, Z);
            }
    }
    double_buffer_zed.put(std::move(new_zed_image));
}


void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2, long timestamp)
{
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Timestamp calculation
//    auto now = std::chrono::system_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    newImage360.timestamp = timestamp;

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();
    newImage360.depth = 3; // assuming color images

    // Establecer el periodo de refresco de la imagen en milisegundos.
//    newImage360.period = getPeriod("Compute");

    // Establecer el periodo real del compute de refresco de la imagen en milisegundos.
    newImage360.period = fps.get_period();

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    if (webotsImageData1 == nullptr or webotsImageData2 == nullptr) return;
    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);
    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_RGBA2RGB);
    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_RGBA2RGB);
    // Plain [cam1 | cam2] concat puts the SEAM between the two 180° captures at the panorama centre —
    // i.e. cam1's own centre (its optical axis) sits a quarter-width off-centre, not in the middle of
    // the final image. Slide the whole composite right by half a camera's width (a circular roll) so
    // cam1 ends up centred and cam2 — which was already spanning the panorama's wrap seam at column 0 —
    // gets split across both edges instead: new[0:w/2]=cam2's tail, new[w/2:w/2+w]=cam1 (centred),
    // new[w/2+w:2w]=cam2's head.
    const int w = _camera1->getWidth();
    const int shift = w / 2;
    cv::Mat img_final = cv::Mat(cv::Size(w * 2, _camera1->getHeight()), CV_8UC3);
    img_2(cv::Rect(w - shift, 0, shift, _camera2->getHeight())).copyTo(img_final(cv::Rect(0, 0, shift, _camera1->getHeight())));
    img_1.copyTo(img_final(cv::Rect(shift, 0, w, _camera1->getHeight())));
    img_2(cv::Rect(0, 0, w - shift, _camera2->getHeight())).copyTo(img_final(cv::Rect(shift + w, 0, w - shift, _camera1->getHeight())));

    // Asignar la imagen RGB 360 al tipo TImage de Robocomp
    newImage360.image.resize(img_final.total()*img_final.elemSize());
    memcpy(&newImage360.image[0], img_final.data, img_final.total()*img_final.elemSize());

    //newImage360.image = rgbImage360;
    newImage360.compressed = false;

    if(pars.delay)
        camera_queue.push(newImage360);

    // Asignamos el resultado final al atributo de clase (si tienes uno).
    double_buffer_360.put(std::move(newImage360));

    //std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;
}

void SpecificWorker::receiving_robotSpeed(webots::Supervisor* _robot, long timestamp)
{
    const double* shadow_position = robotNode->getPosition();
    const double* shadow_orientation = robotNode->getOrientation();
    const double* shadow_velocity = robotNode->getVelocity();
    const float orientation = atan2(shadow_orientation[1], shadow_orientation[0]) - M_PI_2;

    // Webots reports velocities per SIMULATION second; every timestamp we publish is wall-clock.
    // Convert, so a consumer integrating rate*dt(wall) gets the rotation and translation that really
    // happened. See update_sim_wall_rate and the note on sim_wall_rate_ in the header for the
    // measurement that motivated this (6.7% of spurious rotation in room_concept's odometry prior).
    update_sim_wall_rate(_robot->getTime(), timestamp);
#if BRIDGE_WALL_TIME_VELOCITIES
    const double k_wall = sim_wall_rate_;
#else
    const double k_wall = 1.0;
#endif

#if BRIDGE_YAWRATE_DIAG
    // DIAGNOSTIC (2026-08-09): is the angular velocity we publish consistent with this node's OWN
    // rotation? Measured downstream, robot_current_angular_speed integrates to 7.2% more rotation
    // than actually happened: over a pivot that swept four full turns and closed on its starting
    // heading (so the true rotation is exactly 1440 deg), odometry read 1543.4 deg while the command
    // read 1444.0. That was measured against room_concept's map-anchored posterior, which cannot be
    // 7% wrong on ANGLE because a closed rotation pins it. This log removes the localizer from the
    // question entirely: both columns below are supervisor reads from the same cycle, so
    //     d(yaw)/d(sim_time)   vs   wz
    // is a self-contained check of getVelocity()[5]. Logging sim AND wall time also separates a
    // pacing problem (sim slower than wall) from a wrong value, and TIME_STEP is 33 ms against this
    // world's default basicTimeStep of 32, which is worth ruling in or out.
    // Suspicions, in order: aliasing of a ~0.5-1 s oscillation seen in the published rate (3.5-9% of
    // the mean, long same-sign runs) against the consumer's ~50 ms window; a phase error between the
    // sampled velocity and the timestamp; the value simply being wrong.
    {
        static std::ofstream diag("yawrate_diag.csv");
        static bool hdr = false;
        if (not hdr)
        {
            diag.imbue(std::locale::classic());   // never emit a comma decimal separator
            diag << "sim_t,wall_ms,yaw_raw,wz,vx_world,vy_world,sim_wall_rate\n";
            hdr = true;
        }
        diag << _robot->getTime()
             << ',' << timestamp
             << ',' << atan2(shadow_orientation[1], shadow_orientation[0])  // UNSHIFTED: unwrap offline
             << ',' << shadow_velocity[5]
             << ',' << shadow_velocity[0]
             << ',' << shadow_velocity[1]
             << ',' << sim_wall_rate_ << '\n';
    }
#endif

    Eigen::Matrix2f rt_rotation_matrix;
    rt_rotation_matrix << cos(orientation), -sin(orientation),
                          sin(orientation), cos(orientation);

    // Multiply the velocity std::vector by the inverse of the rotation matrix to get the velocity in the robot reference system
    const Eigen::Vector2f shadow_velocity_world(shadow_velocity[1], shadow_velocity[0]);
    // ... and scale to per-WALL-second, because that is the clock our timestamps use. Both linear and
    // angular: the unit mismatch applies identically to translation, it was simply only measured on
    // rotation (which room_concept arbitrates against the room geometry, so it was visible there).
    const Eigen::Vector2f shadow_velocity_local =
        static_cast<float>(k_wall) * (rt_rotation_matrix.inverse() * shadow_velocity_world);
    const double shadow_rot_wall = k_wall * shadow_velocity[5];

    // Noise, if any, is the consumer's business: see RoomConcept.OdometryNoiseFactor.
    RoboCompFullPoseEstimation::FullPoseEuler pose_data;

    // Posición
    pose_data.x = static_cast<float>(shadow_position[0]);  // metros → mm
    pose_data.y = static_cast<float>(shadow_position[1]);
    pose_data.z = static_cast<float>(shadow_position[2]);

    // Orientación (Euler en radianes) 2D
    pose_data.rx = 0.0;
    pose_data.ry = 0.0;
    pose_data.rz = orientation;  // Ángulo Z ya calculado

    // Velocidades en world frame
    pose_data.vx = -shadow_velocity_local(0);
    pose_data.vy = -shadow_velocity_local(1);
    pose_data.vz = 0;
    pose_data.vrx = 0;
    pose_data.vry = 0;
    pose_data.vrz = static_cast<float>(shadow_rot_wall);
    
    // Velocidades en robot frame
    pose_data.adv = shadow_velocity_local(1);  //y
    pose_data.side = -shadow_velocity_local(0);
    pose_data.rot = static_cast<float>(shadow_rot_wall); // per WALL second, see sim_wall_rate_

    pose_data.timestamp = timestamp;

    // Refresh the snapshot served by OmniRobot_getBaseState() while we already hold
    // the freshly read supervisor data, so that RPC never has to query Webots.
    update_base_state(shadow_position, shadow_velocity_local, shadow_rot_wall);

    this->fullposeestimationpub_pubproxy->newFullPose(pose_data);
}

// Runs in the compute() thread only (from receiving_robotSpeed), so clock_pairs_ needs no lock.
void SpecificWorker::update_sim_wall_rate(double sim_seconds, long wall_ms)
{
    // Trailing 10 s window, plus a light EMA. The 2 s window this started with was far too
    // responsive: it ranged 0.88-1.00 frame to frame, which swapped a constant 6.5% bias for a
    // +-6% time-varying one. Measured cost, matched in-place frames: at |omega| 0.8-1.6 rad/s the
    // localizer's prediction early-exit fell 99.6% -> 93.3% and frames over its gate went 0% -> 6.7%,
    // while the margin median did not move -- the signature of variance, not bias. What is being
    // estimated here is how far behind real time the simulation is running, which is a property of
    // machine load and changes over seconds to minutes, so estimating it over 10 s and smoothing is
    // both cheaper and more faithful than tracking every frame.
    clock_pairs_.emplace_back(sim_seconds, wall_ms);
    while (clock_pairs_.size() > 2 and (wall_ms - clock_pairs_.front().second) > 10000)
        clock_pairs_.pop_front();

    const double dwall = (wall_ms - clock_pairs_.front().second) / 1000.0;
    const double dsim  = sim_seconds - clock_pairs_.front().first;

    // Until there is a real span to divide, leave the rate at its last value (1.0 at startup), which
    // is the historical per-sim-second behaviour. A paused simulation lands here too: dsim is 0, so
    // the last good rate is held rather than collapsing the published velocities to zero -- they are
    // zero anyway while paused.
    if (dwall < 0.5 or dsim <= 0.0)
        return;

    const double r = dsim / dwall;
    // Sane band. Outside it something has gone wrong -- a clock jump, a step that blocked for
    // seconds, a resumed pause -- and a bogus scale is worse than none, so hold the last good value.
    if (r > 0.05 and r < 2.0)
    {
        // EMA on top of the window, so the published scale cannot step frame to frame. alpha 0.05 at
        // the ~30 Hz step rate is a time constant of about half a second on top of the 10 s window.
        constexpr double alpha = 0.05;
        sim_wall_rate_ = (1.0 - alpha) * sim_wall_rate_ + alpha * r;
    }
}

void SpecificWorker::update_base_state(const double *position, const Eigen::Vector2f &velocity_local, double rot_velocity)
{
    RoboCompGenericBase::TBaseState state{};

    state.x = static_cast<float>(position[0]);
    state.z = static_cast<float>(position[1]);
    state.alpha = robotRotationField != nullptr ? static_cast<float>(robotRotationField->getSFRotation()[3]) : 0.f;
    state.correctedX = state.x;
    state.correctedZ = state.z;
    state.correctedAlpha = state.alpha;
    state.advVz = velocity_local(1);
    state.advVx = -velocity_local(0);
    state.rotV = static_cast<float>(rot_velocity);
    state.isMoving = std::hypot(state.advVx, state.advVz) > 0.001f or std::fabs(state.rotV) > 0.001f;

    const std::lock_guard<std::mutex> lock(base_state_mutex);
    base_state = state;
}

// Called from servant threads: queue a closure to be run in the Webots thread.
void SpecificWorker::post_webots_task(std::function<void()> task)
{
    const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
    webots_tasks.push_back(std::move(task));
}

// Called from compute(): drain the queue outside the lock, so a task that takes a
// while cannot block servants trying to post new ones.
void SpecificWorker::run_webots_tasks()
{
    std::vector<std::function<void()>> tasks;
    {
        const std::lock_guard<std::mutex> lock(webots_tasks_mutex);
        if (webots_tasks.empty()) return;
        tasks.swap(webots_tasks);
    }
    for (auto &task : tasks)
        task();
}

double SpecificWorker::generate_noise(double stddev)
{
    // Reuse generator to avoid reseeding cost per call.
    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> d(0.0, stddev);
    return d(gen);
}

void SpecificWorker::receiving_lidarData(std::string name, webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue, long timestamp)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    if (rangeImage == nullptr) return;   // no sample yet (first cycle, before any step)
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double minRange = _lidar->getMinRange();
    double maxRange = _lidar->getMaxRange();
    double fov = _lidar->getFov();
    double verticalFov = _lidar->getVerticalFov();

    // Timestamp calculation
//    auto now = std::chrono::system_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Configuration settings
    RoboCompLaser::TLaserData newLaserData;
    RoboCompLaser::LaserConfData newLaserConfData;
    RoboCompLidar3D::TData newLidar3dData;

    // General Lidar values
    newLidar3dData.timestamp = timestamp;
    newLidar3dData.period = fps.get_period();
    newLaserConfData.maxDegrees = fov;
    newLaserConfData.maxRange = maxRange;
    newLaserConfData.minRange = minRange;
    newLaserConfData.angleRes = fov / horizontalResolution;

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    if(name == "helios")
        verticalFov = 2.8;

    newLidar3dData.points.reserve(static_cast<size_t>(horizontalResolution) * static_cast<size_t>(verticalResolution));
    newLaserData.reserve(static_cast<size_t>(horizontalResolution) * static_cast<size_t>(verticalResolution));
    

    // Precompute angles to avoid sin/cos per point.
    std::vector<float> h_angle(horizontalResolution);
    std::vector<float> h_cos(horizontalResolution);
    std::vector<float> h_sin(horizontalResolution);
    for (int i = 0; i < horizontalResolution; ++i)
    {
        float horizontalAngle = static_cast<float>(M_PI - i * newLaserConfData.angleRes - fov / 2.0);
        h_angle[i] = horizontalAngle;
        h_cos[i] = std::cos(horizontalAngle);
        h_sin[i] = std::sin(horizontalAngle);
    }

    std::vector<float> v_angle(verticalResolution);
    std::vector<float> v_cos(verticalResolution);
    std::vector<float> v_sin(verticalResolution);
    for (int j = 0; j < verticalResolution; ++j)
    {
        float verticalAngle = static_cast<float>(M_PI + j * (verticalFov / verticalResolution) - verticalFov / 2.0);
        v_angle[j] = verticalAngle;
        v_cos[j] = std::cos(verticalAngle);
        v_sin[j] = std::sin(verticalAngle);
    }

    // Iterate with horizontal angle outer loop to keep phi-sorted order.
    for (int i = 0; i < horizontalResolution; ++i)
    {
        for (int j = 0; j < verticalResolution; ++j)
        {
            int index = j * horizontalResolution + i;
            static RoboCompLidar3D::TPoint point;


            //distance meters to millimeters
            const float distance = rangeImage[index]; //Meters

            point.phi = h_angle[i];// ángulo horizontal // -x para hacer [PI, -PI] y no [-PI, PI]
            point.theta = v_angle[j];// ángulo vertical

            //Calculate Cartesian co-ordinates and rectify axis positions
            Eigen::Vector3f lidar_point(
                    distance * h_cos[i] * v_cos[j],
                    distance * h_sin[i] * v_cos[j],
                    distance * v_sin[j]);


            if (not (std::isinf(lidar_point.x()) or std::isinf(lidar_point.y()) or std::isinf(lidar_point.z())))
            {
                point.r = lidar_point.norm();  // distancia radial
                if (point.r > 0.2)
                {
                    if ((name == "bpearl" and  point.theta < M_PI) or
                        (name == "helios" and point.theta < 4.10152 and point.theta > 2.8799))//down limit+, uper limit-, horizon line is PI
                        //(name == "helios" and point.theta > 3.4033 and point.theta < 2.1817 ))
                    {
                        point.x = lidar_point.x();
                        point.y = lidar_point.y();
                        point.z = lidar_point.z();

                        point.distance2d = std::hypot(lidar_point.x(),lidar_point.y());  // distancia en el plano xy

                        newLidar3dData.points.push_back(point);
                        newLaserData.push_back(RoboCompLaser::TData{.angle=point.phi, .dist = point.distance2d});
                    }
                }
        
            }
        }
    }

    laserData = newLaserData;
    laserDataConf = newLaserConfData;

    //Is it necessary to use two lidar queues? One for each lidaR?
    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
}
void SpecificWorker::receiving_cameraRGBData(webots::Camera* _camera, long timestamp)
{
    RoboCompCameraRGBDSimple::TImage newImage{};

    // Se establece el periodo de refresco de la imagen en milisegundos.
//    newImage.period = getPeriod("Compute");
    newImage.period = fps.get_period();

    // Timestamp calculation
//    auto now = std::chrono::system_clock::now();
//    auto duration = now.time_since_epoch();
//    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    newImage.alivetime = timestamp;

    // Obtener la resolución de la imagen.
    newImage.width = _camera->getWidth();
    newImage.height = _camera->getHeight();
    newImage.depth = 3;  // assuming color images

    const unsigned char* webotsImageData = _camera->getImage();
    cv::Mat imageMatBGRA(newImage.height, newImage.width, CV_8UC4, (void*)webotsImageData);
    cv::Mat imageMatBGR;
    cv::cvtColor(imageMatBGRA, imageMatBGR, cv::COLOR_BGRA2BGR);

    newImage.image.resize(imageMatBGR.total() * imageMatBGR.channels());
    std::memcpy(newImage.image.data(), imageMatBGR.data, newImage.image.size());
    newImage.compressed = false;

    // Asignamos el resultado final al atributo de clase
    this->cameraImage = newImage;
}
// void SpecificWorker::receiving_depthImageData(webots::RangeFinder* _rangeFinder, long timestamp)
// {
//     RoboCompCameraRGBDSimple::TDepth newDepthImage{};
//
//     // Se establece el periodo de refresco de la imagen en milisegundos.
//     newDepthImage.period = fps.get_period();
//
//     // Obtener la resolución de la imagen de profundidad.
//     newDepthImage.width = _rangeFinder->getWidth();
//     newDepthImage.height = _rangeFinder->getHeight();
//     newDepthImage.depthFactor = 1;
//     newDepthImage.compressed = false;
//
//     // Obtener la imagen de profundidad
//     const float* webotsDepthData = _rangeFinder->getRangeImage();
//
//     // Accedemos a cada depth value y le aplicamos un factor de escala.
//     const int imageElementCount = newDepthImage.width * newDepthImage.height;
//     newDepthImage.depth.resize(static_cast<size_t>(imageElementCount) * sizeof(float));
//
//     unsigned char* out = newDepthImage.depth.data();
//     for (int i = 0; i < imageElementCount; i++)
//     {
//         // Este es el factor de escala a aplicar.
//         float scaledValue = webotsDepthData[i] * 10;
//         std::memcpy(out + static_cast<size_t>(i) * sizeof(float), &scaledValue, sizeof(float));
//     }
//
//     // Asignamos el resultado final al atributo de clase
//     this->depthImage = newDepthImage;
// }

#pragma endregion Data-Catching Methods

#pragma region Camera360

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if(pars.delay)
    {
        if(camera_queue.full())
            return camera_queue.back();
    }

    return double_buffer_360.get_idemp();
}

#pragma endregion Camera360

#pragma region CameraRGBDSimple

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return double_buffer_zed.get_idemp();
}

// The partial getters project out the one member they return: copying the whole
// TRGBD (image + depth + points) to hand back a third of it costs several MB per call.
RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.depth; });
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.image; });
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return double_buffer_zed.get_idemp_with([](const auto &rgbd) { return rgbd.points; });
}

#pragma endregion CamerRGBDSimple

#pragma region Lidar

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserData;
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserDataConf;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    return laserData;
}
RoboCompLidar3D::TColorCloudData SpecificWorker::Lidar3D_getColorCloudData()
{
	RoboCompLidar3D::TColorCloudData ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    if(name == "helios") {
        if(pars.delay && helios_delay_queue.full())
            return helios_delay_queue.back();
        else
            return double_buffer_helios.get_idemp();
    }
    else if(name == "bpearl")
    {
        if(pars.delay && pearl_delay_queue.full())
            return pearl_delay_queue.back();
        else
            return double_buffer_pearl.get_idemp();
    }
    else
    {
        std::cout << "Getting data from a not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << std::endl;
        return RoboCompLidar3D::TData();
    }
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance, int decimationDegreeFactor)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataWithThreshold2d");
    return RoboCompLidar3D::TData();
}

RoboCompLidar3D::TDataCategory SpecificWorker::Lidar3D_getLidarDataByCategory(RoboCompLidar3D::TCategories categories, Ice::Long timestamp)
{
	RoboCompLidar3D::TDataCategory ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
	return ret;
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataArrayProyectedInImage");
    return RoboCompLidar3D::TDataImage();
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataProyectedInImage(std::string name)
{
	#ifdef HIBERNATION_ENABLED
		hibernation = true;
	#endif
	RoboCompLidar3D::TData ret{};
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("Lidar3D_getLidarDataProyectedInImage");
    return ret;
}


#pragma endregion Lidar

#pragma region OmniRobot

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_getBasePose");
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Served from the snapshot refreshed by compute(); no Webots call here.
    const std::lock_guard<std::mutex> lock(base_state_mutex);
    state = base_state;
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_resetOdometer");
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_setOdometer");
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Only queue the command. Webots buffers actuator requests until the next
    // wb_robot_step() anyway, so applying it in compute() costs no extra actuation
    // latency and keeps this call at memory-copy speed.
    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {advx, advz, rot};
    speed_command_pending = true;
}

void SpecificWorker::OmniRobot_stopBase()
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(speed_command_mutex);
    speed_command = {0.f, 0.f, 0.f};
    speed_command_pending = true;
}

// Runs in the compute() thread, just before robot->step().
void SpecificWorker::apply_pending_speed_command()
{
    SpeedCommand cmd;
    {
        const std::lock_guard<std::mutex> lock(speed_command_mutex);
        if (not speed_command_pending) return;
        cmd = speed_command;
        speed_command_pending = false;
    }

    const double advx     = cmd.advx * 0.001;   // mm/s -> m/s
    const double advz_cmd = cmd.advz * 0.001;

    // Asymmetric-wheelbase compensation. The formula below is the symmetric mecanum form, so it
    // rotates the robot about the WHEEL CENTROID rather than about the frame origin the caller means.
    // MEASURED 08-09 by fitting p_k + R(theta_k)*d = c over a clean pivot (139 deg swept, 4.8 cm of
    // travel, 4.6 mm mean / 8.2 mm max residual): |d| = 2.72 cm, matching the 3.0 cm the proto's wheel
    // layout predicts, and lying along ONE axis (the other component was 0.14 cm).
    //
    // The offset came out along the robot frame's LATERAL axis, so the origin's induced velocity
    // during a pivot is LONGITUDINAL -- which is why this term is on advz and not advx as first
    // written. Still disabled: WHEEL_CENTROID_OFFSET is 0.0 because the SIGN depends on conventions
    // I could not pin by reading (adv/side to robot x/y differs between the two integrators in
    // room_concept.cpp:3967 and :4013, and cmd.rot's sign against local z is asserted only by a
    // stale-looking "CW positive" comment above). Getting it backwards doubles the error to 5.4 cm.
    // See specificworker.h for the one-pivot trial that settles it.
    const double advz = advz_cmd + WHEEL_CENTROID_OFFSET * cmd.rot;

    const double speeds[4] = {1.0 / WHEEL_RADIUS * (advz - advx + (LX + LY) * cmd.rot),
                              1.0 / WHEEL_RADIUS * (advz + advx - (LX + LY) * cmd.rot),
                              1.0 / WHEEL_RADIUS * (advz + advx + (LX + LY) * cmd.rot),
                              1.0 / WHEEL_RADIUS * (advz - advx - (LX + LY) * cmd.rot)};

    for (int i = 0; i < 4; i++)
        motors[i]->setVelocity(speeds[i]);
}

#pragma endregion OmniRobot

#pragma region VisualElements

// Runs in the compute() thread, right after parseHumanObjects() has refreshed the map.
void SpecificWorker::update_visual_objects()
{
    RoboCompVisualElements::TObjects objectsList;

    for (const auto &entry : humanObjects)
    {
        webots::Node *node = entry.second.node;
        if (node == nullptr) continue;

        const double *position = node->getPosition();

        RoboCompVisualElements::TObject object;
        object.id = entry.first;
        object.x = position[0];
        object.y = position[1];

        objectsList.objects.push_back(object);
    }

    const std::lock_guard<std::mutex> lock(visual_objects_mutex);
    visual_objects = std::move(objectsList);
}

RoboCompVisualElements::TObjects SpecificWorker::VisualElements_getVisualObjects(RoboCompVisualElements::TObjects objects)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(visual_objects_mutex);
    return visual_objects;
}
void SpecificWorker::VisualElements_setVisualObjects(RoboCompVisualElements::TObjects objects)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    // Implement CODE
}

#pragma endregion VisualElements

#pragma region Webots2Robocomp Methods

void SpecificWorker::humansMovement()
{
    if(!humanObjects.empty())
        for (auto& human : humanObjects)
        {
            qInfo() << "ID" << human.first;

            qInfo() << "Moving human " << human.first << human.second.path.size();
            moveHumanToNextTarget(human.first);

        }
}

void SpecificWorker::moveHumanToNextTarget(int humanId)
{
    webots::Node *humanNode = humanObjects[humanId].node;

    if(humanNode == nullptr)
    {
        qInfo() << "Human not found";
        return;
    }
    humanObjects[humanId].node->getPosition() ;
    double velocity[3];

    if(humanObjects[humanId].path.empty())
    {
        qInfo() <<"Set velocity to 0, path empty";
        velocity[0] = 0.f;
        velocity[1] = 0.f;
        velocity[2] = 0.f;
    }
    else
    {
        const double *position = humanNode->getPosition();

        Eigen::Vector3d currentTarget {humanObjects[humanId].path.front().x - position[0], humanObjects[humanId].path.front().y - position[1] , 0};

        //Erase path if .norm < d = 300mm
        if(currentTarget.norm() < 0.3)
            humanObjects[humanId].path.erase(humanObjects[humanId].path.begin());

        //Print currentTarget std::vector values
        qInfo() << "TARGET:" << currentTarget.x() << currentTarget.y() << currentTarget.z();

        currentTarget.normalize();

        velocity[0] = currentTarget.x();
        velocity[1] = currentTarget.y();
        velocity[2] = 0.f;
        //Print velocity std::vector values
        qInfo() << "SPEED:" << velocity[0] << velocity[1] << velocity[2];

    }

    humanNode->setVelocity(velocity);
}

void SpecificWorker::Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    // Deferred: this both reads the supervisor and mutates humanObjects, which
    // compute() rewrites. Running it there makes the call return immediately and
    // removes the race on the map.
    post_webots_task([this, humanId, path = std::move(path)]
                     { set_path_to_human(humanId, path); });
}

void SpecificWorker::set_path_to_human(int humanId, const RoboCompGridder::TPath &path)
{
    //static bool overwrite_path = false;

    if(humanObjects[humanId].node == nullptr)
    {
        qInfo() << "Human not found";
        return;
    }

    if(humanObjects[humanId].path.empty() and path.size() > 3)
    {
        RoboCompGridder::TPath transformed_path;

        if(robotTranslationField == nullptr or robotRotationField == nullptr) return;
        auto x = robotTranslationField->getSFVec3f()[0] * 1000;
        auto y = robotTranslationField->getSFVec3f()[1] * 1000;
        auto rot = robotRotationField->getSFRotation();

        Eigen::Vector3d axis(rot[0], rot[1], rot[2]);
        Eigen::AngleAxisd angleAxis(rot[3], axis.normalized());
        Eigen::Quaterniond quaternion(angleAxis);

        Eigen::Vector3d eulerAngles = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

//    //Get transform matrix

        auto tf = create_affine_matrix(eulerAngles.x(), eulerAngles.y(), eulerAngles.z(), Eigen::Vector3d {x, y, 0});

        //    //Print tf matrix values
//    qInfo() << "Transform matrix" << tf(0,0) << tf(0,1) << tf(0,2) << tf(0,3) << tf(1,0) << tf(1,1) << tf(1,2) << tf(1,3) << tf(2,0) << tf(2,1) << tf(2,2) << tf(2,3) << tf(3,0) << tf(3,1) << tf(3,2) << tf(3,3);

        for(auto &p : path)
        {
            Eigen::Vector2f tf_point = (tf.matrix() * Eigen::Vector4d(p.y , -p.x, 0.0, 1.0)).head(2).cast<float>();
            transformed_path.emplace_back(RoboCompGridder::TPoint(tf_point.x() / 1000, tf_point.y() / 1000 , 100.0));
            qInfo() << __FUNCTION__ << "point?" << tf_point.x() << tf_point.y();
        }

        humanObjects[humanId].path = transformed_path;
    }

}

void SpecificWorker::Webots2Robocomp_resetWebots()
{
//implementCODE
    
}

void SpecificWorker::Webots2Robocomp_setDoorAngle(std::string DEF, float angle)
{
    last_read.store(std::chrono::high_resolution_clock::now());
    // The DEF is copied into the task: the caller's string is gone by the time the supervisor thread
    // runs this, and the lookup itself must happen there (Webots node access is not thread-safe).
    post_webots_task([this, DEF = std::move(DEF), angle] { setDoorAperture(DEF, angle); });
}

RoboCompWebots2Robocomp::Quaternion axisAngleToQuaternion(const webots::Field &rotation) {
    RoboCompWebots2Robocomp::Quaternion q;

    double rx = rotation.getSFRotation()[0];
    double ry = rotation.getSFRotation()[1];
    double rz = rotation.getSFRotation()[2];
    const double angle = rotation.getSFRotation()[3];

    // Aseguramos que el eje esté normalizado (por seguridad)
    // En Webots generalmente ya viene normalizado, pero esto evita errores.
    if (const double norm = std::sqrt(rx*rx + ry*ry + rz*rz); norm > 0) {
        rx /= norm;
        ry /= norm;
        rz /= norm;
    }

    // Fórmulas de ángulo medio
    const double halfAngle = angle * 0.5;
    const double sinHalf = std::sin(halfAngle);

    q.w = std::cos(halfAngle);
    q.x = rx * sinHalf;
    q.y = ry * sinHalf;
    q.z = rz * sinHalf;

    return q;
}

// Runs in the compute() thread: resolve any newly requested DEF, then refresh every
// tracked pose. The Webots calls happen with no lock held, so servants reading the
// cache are never blocked by them.
void SpecificWorker::update_object_poses()
{
    std::vector<std::string> defs;
    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        if (object_poses.empty()) return;
        defs.reserve(object_poses.size());
        for (const auto &[def, _] : object_poses)
            defs.push_back(def);
    }

    std::vector<TrackedObject> refreshed(defs.size());
    for (size_t i = 0; i < defs.size(); ++i)
    {
        const std::string &def = defs[i];
        refreshed[i].resolved = true;   // resolved means "compute() has tried", not "found"

        auto handles = object_handles.find(def);
        if (handles == object_handles.end())
        {
            TrackedObjectHandles h;
            if (webots::Node *node = robot->getFromDef(def); node != nullptr)
            {
                h.translation = node->getField("translation");
                h.rotation = node->getField("rotation");
                if (h.translation == nullptr or h.rotation == nullptr)
                    std::cout << "Object node dont have postion or rotation" << std::endl;
            }
            else
                std::cout << "Object with DEF:" << def << "not found" << std::endl;

            handles = object_handles.emplace(def, h).first;
        }

        if (handles->second.translation == nullptr or handles->second.rotation == nullptr)
            continue;   // leaves a zeroed pose, as the original code returned

        const double *position = handles->second.translation->getSFVec3f();
        refreshed[i].pose.position = RoboCompWebots2Robocomp::Vector3{
            static_cast<float>(position[0] * 1000),
            static_cast<float>(position[1] * 1000),
            static_cast<float>(position[2] * 1000),
        };
        refreshed[i].pose.orientation = axisAngleToQuaternion(*handles->second.rotation);
    }

    {
        const std::lock_guard<std::mutex> lock(object_poses_mutex);
        for (size_t i = 0; i < defs.size(); ++i)
            object_poses[defs[i]] = refreshed[i];
    }
    object_poses_cv.notify_all();
}

RoboCompWebots2Robocomp::ObjectPose SpecificWorker::Webots2Robocomp_getObjectPose(std::string DEF)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    std::unique_lock<std::mutex> lock(object_poses_mutex);
    TrackedObject &tracked = object_poses[DEF];   // registers the DEF on first use
    if (tracked.resolved)
        return tracked.pose;

    // First ever request for this DEF: nothing cached yet, so wait for one compute()
    // cycle to resolve it. Bounded, and only ever paid once per DEF.
    hibernation = true;   // make sure the loop is at full rate while we wait
    object_poses_cv.wait_for(lock, std::chrono::seconds(1), [&tracked] { return tracked.resolved; });
    if (not tracked.resolved)
        std::cout << "Timed out resolving object with DEF:" << DEF << std::endl;
    return tracked.pose;
}

#pragma endregion Webots2Robocomp Methods

#pragma region JoystickAdapter

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    // Declaration of the structure to be filled
    float side=0, adv=0, rot=0;
    /*
    // Iterate through the std::list of buttons in the data structure
    for (RoboCompJoystickAdapter::ButtonParams button : data.buttons) {
        // Currently does nothing with the buttons
    }
    */

    // Iterate through the std::list of axes in the data structure
    for (RoboCompJoystickAdapter::AxisParams axis : data.axes)
    {
        // Process the axis according to its name
        if(axis.name == "rotate")
            rot = axis.value;
        else if (axis.name == "advance")
            adv = axis.value;
        else if (axis.name == "side")
            side = axis.value;
        else
            std::cout << "[ JoystickAdapter ] Warning: Using a non-defined axes (" << axis.name << ")." << std::endl;
    }
    if(pars.do_joystick)
        OmniRobot_setSpeedBase(side, adv, rot);
}

#pragma endregion JoystickAdapter

#pragma region IMU

// Runs in the compute() thread. The accelerometer and gyro are sampled at the
// compute period, so a per-cycle snapshot loses no information.
void SpecificWorker::update_imu_data(long timestamp)
{
    RoboCompIMU::DataImu data{};

    if (accelerometer != nullptr)
        if (const double *values = accelerometer->getValues(); values != nullptr)
            data.acc = {(float)values[0], (float)values[1], (float)values[2], timestamp};

    if (gyroscope != nullptr)
        if (const double *values = gyroscope->getValues(); values != nullptr)
            data.gyro = {(float)values[0], (float)values[1], (float)values[2], timestamp};

    if (robotNode != nullptr)
    {
        const auto [roll, pitch, yaw] = rotationMatrixToEulerZYX(robotNode->getOrientation());
        data.rot = {roll, pitch, yaw, timestamp};
    }

    data.temperature = 0.f;

    const std::lock_guard<std::mutex> lock(imu_mutex);
    imu_data = data;
}

RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.acc;
}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.gyro;
}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data;
}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
    RoboCompIMU::Magnetic ret{};
    printNotImplementedWarningMessage(__FUNCTION__);
    return ret;
}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
    last_read.store(std::chrono::high_resolution_clock::now());
    const std::lock_guard<std::mutex> lock(imu_mutex);
    return imu_data.rot;
}

std::tuple<float, float, float> SpecificWorker::rotationMatrixToEulerZYX(const double* R)
{
    float roll, pitch, yaw;

    if (R[6] < 1.0) {
        if (R[6] > -1.0) {
            pitch = std::asin(-R[6]);
            roll  = std::atan2(R[7], R[8]);
            yaw   = std::atan2(R[3], R[0]);
        } else {
            // R[6] == -1
            pitch = M_PI / 2.0;
            roll  = -std::atan2(-R[1], R[4]);
            yaw   = 0.0;
        }
    } else {
        // R[6] == +1
        pitch = -M_PI / 2.0;
        roll  = std::atan2(-R[1], R[4]);
        yaw   = 0.0;
    }

    return {roll, pitch, yaw};
}

void SpecificWorker::IMU_resetImu()
{
    printNotImplementedWarningMessage(__FUNCTION__);
}

#pragma endregion IMU

void SpecificWorker::printNotImplementedWarningMessage(std::string functionName)
{
    std::cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}

Matrix4d SpecificWorker::create_affine_matrix(double a, double b, double c, Vector3d trans)
{

    Transform<double, 3, Eigen::Affine> t;
    t = Translation<double, 3>(trans);
    t.rotate(AngleAxis<double>(a, Vector3d::UnitX()));
    t.rotate(AngleAxis<double>(b, Vector3d::UnitY()));
    t.rotate(AngleAxis<double>(c, Vector3d::UnitZ()));
    return t.matrix();
}

void SpecificWorker::parseHumanObjects() {
    webots::Node* crowdNode = robot->getFromDef("CROWD");
    if(!crowdNode){

        static bool ErrorFlag = false;
        if(!ErrorFlag){
            qInfo() << "CROWD Node not found.";
            ErrorFlag = true;
        }

        return;
    }

    webots::Field* childrenField = crowdNode->getFieldByIndex(0);
    for (int i = 0; i < childrenField->getCount(); ++i)
    {
        std::string nodeDEF = childrenField->getMFNode(i)->getDef();
        if(nodeDEF.find("HUMAN_") != std::string::npos)
            humanObjects[i].node = childrenField->getMFNode(i);
    }
}

// Open/close the hinged door named by its Webots DEF. Runs on the supervisor thread (posted by
// Webots2Robocomp_setDoorAngle), which is the only thread allowed to touch webots::Node.
//
// The DEF is a PARAMETER now, not the single hard-wired CONTROLLABLE_DOOR: a scene has more than one
// door, and only the caller knows which one it means. An unknown DEF, or a node with no `position`
// field, is reported and ignored — a request to move a door that is not there must fail loudly and
// change nothing, never silently move a different one.
void SpecificWorker::setDoorAperture(const std::string &DEF, float _aperture) {

    if (DEF.empty()) {
        std::cerr << "[Error] setDoorAngle called with an empty DEF.\n";
        return;
    }

    // Resolved once per DEF and cached: getFromDef walks the scene tree, and this can be called every
    // control cycle while an affordance servos the door open. A null result is cached too — it means
    // "this scene has no such node", which does not change while the world is loaded.
    auto it = doorNodes.find(DEF);
    if (it == doorNodes.end())
        it = doorNodes.emplace(DEF, robot->getFromDef(DEF)).first;
    webots::Node *door = it->second;
    if (!door) {
        std::cerr << "[Error] no node with DEF '" << DEF << "' in this world — door not moved.\n";
        return;
    }

    // Clamp the input value to the range [-pi, 0]
    float minAperture = -static_cast<float>(M_PI);
    float maxAperture = 0.0f;

    if (_aperture < minAperture || _aperture > maxAperture) {
        std::cout << "[Warning] The aperture value " << _aperture
                  << " is out of the allowed range ["
                  << minAperture << ", " << maxAperture
                  << "]. It will be clamped automatically.\n";
    }

    float aperture = std::clamp(_aperture, minAperture, maxAperture);

    // Get the "position" field of the door node. Present on a HingeJoint's jointParameters — a DEF that
    // names, say, the door's Solid instead of its joint resolves fine above and fails here, so the
    // message names the DEF: that is the difference between "wrong door" and "wrong node in the door".
    webots::Field *positionField = door->getField("position");
    if (!positionField) {
        std::cerr << "[Error] node '" << DEF << "' has no 'position' field — it is not a hinge joint "
                     "(is this the door's Solid rather than its HingeJoint?).\n";
        return;
    }

    // Set the door position
    positionField->setSFFloat(aperture);
    std::cout << "[Info] Door '" << DEF << "' aperture set to " << aperture << " rad.\n";
}

// ─────────────────────────────────────────────────────────────────────────────────────────────
// DoorControl — the simulator as one provider among several
//
// Nothing below reports whether a door IS open: it reports what this provider did about a
// REQUEST. The caller confirms the door moved by looking at it. That is not a limitation of the
// simulator — it is the contract that lets the same caller talk to a relay in a real apartment,
// or to a component that asks a person, without changing a line.
// ─────────────────────────────────────────────────────────────────────────────────────────────

// Runs in the compute() (Webots) thread. Refreshes the advertised door list: for each hinge DEF
// named in config, where is it. The caller associates these poses with its own fitted apertures,
// so the ids here stay opaque — they happen to be Webots DEFs, and nothing outside may assume it.
void SpecificWorker::update_door_control()
{
    if (door_control_defs.empty() or robot == nullptr) return;

    RoboCompDoorControl::DoorRefList doors;
    doors.reserve(door_control_defs.size());
    for (const auto &def : door_control_defs)
    {
        auto it = doorNodes.find(def);
        if (it == doorNodes.end())
            it = doorNodes.emplace(def, robot->getFromDef(def)).first;
        if (it->second == nullptr) continue;    // named in config but not in this world

        // A hinge's jointParameters carries `anchor`, not `translation`: the pose that matters to a
        // caller is the door's PLACE, so walk up to the parent Solid, which is the node that has one.
        webots::Node *placed = it->second;
        webots::Field *t = placed->getField("translation");
        for (int hop = 0; t == nullptr and hop < 3; ++hop)
        {
            placed = placed->getParentNode();
            if (placed == nullptr) break;
            t = placed->getField("translation");
        }
        if (t == nullptr) continue;

        const double *p = t->getSFVec3f();
        RoboCompDoorControl::DoorRef ref;
        ref.id = def;
        ref.pose.x = static_cast<float>(p[0] * 1000.0);      // mm, to match getObjectPose
        ref.pose.y = static_cast<float>(p[1] * 1000.0);
        ref.pose.angle = 0.f;
        if (webots::Field *r = placed->getField("rotation"); r != nullptr)
        {
            const double *rot = r->getSFRotation();
            ref.pose.angle = static_cast<float>(rot[2] >= 0 ? rot[3] : -rot[3]);   // z-axis only
        }
        ref.width = 1000.f;
        doors.push_back(ref);
    }

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    door_control_doors = std::move(doors);
}

RoboCompDoorControl::Capabilities SpecificWorker::DoorControl_getCapabilities()
{
    last_read.store(std::chrono::high_resolution_clock::now());

    RoboCompDoorControl::Capabilities caps;
    const std::lock_guard<std::mutex> lock(door_control_mutex);
    caps.doors = door_control_doors;
    caps.canActuate = not caps.doors.empty();
    caps.continuousAngle = true;          // a supervisor teleport can hold any angle
    caps.requiresHuman = false;           // nobody is being asked a favour here
    caps.typicalLatencySec = 0.1f;        // one compute() period: the task runs before the next step
    return caps;
}

RoboCompDoorControl::RequestAck SpecificWorker::DoorControl_requestDoor(RoboCompDoorControl::DoorRequest req)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    RoboCompDoorControl::RequestAck ack;
    ack.requestId = 0;
    ack.accepted = false;
    ack.refusal = RoboCompDoorControl::RefusalReason::UnknownDoor;
    ack.expectedLatencySec = 0.1f;

    // Resolve the door. The caller normally quotes an id we advertised; if it quotes none (or one we
    // do not know) fall back to matching by PLACE, which is the resolution rule that survives when
    // the provider is swapped for one that never heard of Webots DEFs.
    std::string def;
    {
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        for (const auto &d : door_control_doors)
            if (d.id == req.doorId) { def = d.id; break; }

        if (def.empty())
        {
            float best = 400.f;   // mm; beyond this it is a different door, not a noisy estimate
            for (const auto &d : door_control_doors)
            {
                const float dx = d.pose.x - req.door.pose.x, dy = d.pose.y - req.door.pose.y;
                if (const float dist = std::sqrt(dx * dx + dy * dy); dist < best)
                    { best = dist; def = d.id; }
            }
        }
    }
    if (def.empty())
    {
        std::cerr << "[DoorControl] no door matches id '" << req.doorId << "' or place ("
                  << req.door.pose.x << ", " << req.door.pose.y << ") — refusing.\n";
        return ack;   // UnknownDoor: telling the caller to stop asking is more useful than silence
    }

    // setDoorAperture clamps to [-pi, 0]; 0 is flush. "Open" without a usable hint means 90 degrees.
    float target = 0.f;
    if (req.action == RoboCompDoorControl::DoorAction::OpenDoor)
        target = (req.apertureHint < -1e-3f) ? req.apertureHint
                                             : -static_cast<float>(M_PI) / 2.f;

    int id = 0;
    {
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        id = door_control_next_id++;
        door_control_requests[id] = RoboCompDoorControl::RequestStatus{
            id, RoboCompDoorControl::RequestState::Queued, RoboCompDoorControl::RefusalReason::NoRefusal};
    }

    post_webots_task([this, id, def, target]
    {
        setDoorAperture(def, target);
        const std::lock_guard<std::mutex> lock(door_control_mutex);
        if (auto it = door_control_requests.find(id); it != door_control_requests.end()
            and it->second.state != RoboCompDoorControl::RequestState::Aborted)
            it->second.state = RoboCompDoorControl::RequestState::Delivered;
    });

    ack.requestId = id;
    ack.accepted = true;
    ack.refusal = RoboCompDoorControl::RefusalReason::NoRefusal;
    return ack;
}

RoboCompDoorControl::RequestStatus SpecificWorker::DoorControl_getRequestStatus(int requestId)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    if (auto it = door_control_requests.find(requestId); it != door_control_requests.end())
        return it->second;

    // An id we never issued. Aborted rather than a throw: a caller polling across a provider
    // restart should be told to give up on this request, not handed an exception to special-case.
    return RoboCompDoorControl::RequestStatus{requestId, RoboCompDoorControl::RequestState::Aborted,
                                             RoboCompDoorControl::RefusalReason::NoRefusal};
}

void SpecificWorker::DoorControl_cancelRequest(int requestId)
{
    last_read.store(std::chrono::high_resolution_clock::now());

    const std::lock_guard<std::mutex> lock(door_control_mutex);
    if (auto it = door_control_requests.find(requestId); it != door_control_requests.end()
        and it->second.state == RoboCompDoorControl::RequestState::Queued)
        it->second.state = RoboCompDoorControl::RequestState::Aborted;
    // Already delivered: the teleport happened, and pretending otherwise would be a lie.
}


/**************************************/
// From the RoboCompFullPoseEstimationPub you can publish calling this methods:
// RoboCompFullPoseEstimationPub::void this->fullposeestimationpub_pubproxy->newFullPose(RoboCompFullPoseEstimation::FullPoseEuler pose)

/**************************************/
// From the RoboCompCamera360RGB you can use this types:
// RoboCompCamera360RGB::TRoi
// RoboCompCamera360RGB::TImage

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompIMU you can use this types:
// RoboCompIMU::Acceleration
// RoboCompIMU::Gyroscope
// RoboCompIMU::Magnetic
// RoboCompIMU::Orientation
// RoboCompIMU::DataImu

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TDataImage
// RoboCompLidar3D::TData
// RoboCompLidar3D::TDataCategory
// RoboCompLidar3D::TColorCloudData

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompVisualElements you can use this types:
// RoboCompVisualElements::TRoi
// RoboCompVisualElements::TObject
// RoboCompVisualElements::TObjects

/**************************************/
// From the RoboCompWebots2Robocomp you can use this types:
// RoboCompWebots2Robocomp::Vector3
// RoboCompWebots2Robocomp::Quaternion
// RoboCompWebots2Robocomp::ObjectPose

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

