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
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <mutex>
#include <tuple>
#include <condition_variable>
#include <functional>
#include <map>
#include <string>
#include <deque>
#include <utility>
#include <vector>

using namespace Eigen;
using namespace std;
// robot geometry
// ── Base geometry: these MUST match webots-shadow/protos/Shadow.proto ────────────────────────────
// The proto puts the four wheel centres at (x = ±0.21, y ∈ {-0.10, +0.16}) in the robot's local
// frame, where x is lateral and y longitudinal (the bridge maps forward = local y, side = local x —
// see update_base_state()). So the half-extents are 0.13 longitudinal and 0.21 lateral.
#define WHEEL_RADIUS 0.05   // matches Interior/ExteriorWheel.proto `radius 0.05`
#define LX 0.130  // longitudinal half-distance, wheel centre to centre / 2 [m]  (proto: (0.16-(-0.10))/2)
#define LY 0.210  // lateral half-distance [m]  (proto: |±0.21|)
//
// LY WAS 0.237, and that single constant was measurably wrong. Only rotation uses (LX+LY), so it
// showed up as the base turning FASTER than commanded while translation stayed correct:
//     bridge (LX+LY) 0.372 / true 0.340 = 1.094 predicted over-rotation
//     measured over 15 in-place turns against room_concept's SDF posterior: 1.109  (1.3% apart)
// Measured by integrating the localizer's own cmd_dth / meas_dth columns, not by re-integrating the
// logged velocity columns — the command is a sparse stepwise signal and sampling it at frame times
// exaggerates its error badly (it read as 34% that way instead of 11%).
//
// Downstream cost while it was wrong: room_concept's command motion prior under-predicted rotation,
// so the SDF had to drag heading further into every turn, which suppressed the localizer's
// prediction early-exit during rotation. Anything doing OPEN-LOOP rotation from a commanded omega
// (a controller's final-orientation move) was simply 9% short.
//
// ── Known, NOT yet corrected: the wheelbase is asymmetric ─────────────────────────────────────────
// Those y offsets are -0.10 and +0.16, so the wheel centroid sits 0.03 m AHEAD of the proto origin,
// while the inverse kinematics below is the standard symmetric-mecanum form and therefore rotates
// the robot about that centroid, not about the frame origin the caller means. A commanded pure
// rotation consequently translates the origin at |v| = 0.03·|omega| — about 4.5 cm/s at 1.5 rad/s,
// tracing a 3 cm-radius arc instead of pivoting in place.
//
// MEASURED 08-09, so this is no longer a prediction. Fitting p_k + R(theta_k)·d = c over a clean
// pivot in room_concept's etc/pose_trace.csv — 139° swept, only 4.8 cm of travel, 4.6 mm mean and
// 8.2 mm max residual — gives |d| = 2.72 cm against the 3.0 cm predicted, with 2.72 of it on one
// axis and 0.14 on the other. A four-turn stretch in the same run agrees in direction. So the
// asymmetry is real, and its magnitude is 0.027 m rather than the nominal 0.03.
//
// The correction is a twist transport, no change to the kinematics needed: rotating about the origin
// is rotating about the centroid plus one translational term. The measured offset lies along the
// robot frame's LATERAL axis, so the origin's induced velocity during a pivot is LONGITUDINAL and
// the term goes on advz (see apply_pending_speed_command) — it was on advx in the first draft of
// this, which was wrong.
//
// What is still NOT established is the SIGN, because it rides on conventions that reading cannot
// settle: room_concept maps (adv, side) to robot (x, y) one way at room_concept.cpp:3967 and the
// other way at :4013, and cmd.rot's sign against local z rests on a "CW positive" comment that looks
// stale. Backwards doubles the error to 5.4 cm, so this ships DISABLED at 0.0.
//
// ONE PIVOT SETTLES IT. A pivot means rotation in place: zero commanded translation, net travel
// under ~15 cm, and the fit residual under 1 cm (an arc gives 30-130 mm and is useless here).
//   1. set WHEEL_CENTROID_OFFSET to +0.027
//   2. drive one pivot of >= 180°
//   3. re-fit d. Toward 0 confirms it; growing to ~5.4 cm means flip the constant's sign — flip the
//      constant, not the expression, so this comment stays true.
#define WHEEL_CENTROID_OFFSET 0.0  // [m] 0.0 = off; the measured value is 0.027 (verify the sign)

// Publish velocities in per-WALL-second units instead of the true per-SIM-second ones. NOW OFF.
//
// This existed because there was no simulation clock on the wire. Webots reports velocities per
// simulation second, every timestamp we emitted was wall-clock, and a consumer integrating
// rate*dt(wall) therefore over-counted by however far the sim ran behind real time -- measured at
// 6.7% of spurious rotation in room_concept's odometry prior (commit 5968a65). Pre-scaling the rate
// by sim_wall_rate_ made the two agree.
//
// It is off now because FullPoseEuler and the IMU structs carry `simTimestamp`, so the mismatch it
// patched no longer exists: a consumer integrates the true rate over the clock that rate is measured
// in. Three reasons that is better than the scaling:
//   - It makes simulation and hardware SYMMETRIC. On the real robot sim time IS wall time, so raw
//     velocity against its own stamp is correct in both cases and the consumer needs no special case.
//   - The published number becomes a physical fact -- the robot's actual speed in its own world --
//     rather than one that only means anything paired with a wall stamp.
//   - It takes sim_wall_rate_ out of the velocity path, and with it the variance documented below:
//     the estimator's earlier 2 s window swapped a constant 6.5% bias for a +-6% time-varying one and
//     measurably cost the localizer's prediction early-exit. A jittering scale factor is a poor thing
//     to have inside anything that integrates.
//
// sim_wall_rate_ is still computed and still used, but only for the acquisition-time arithmetic in
// acquisition_wall_ms(), where converting an age in sim-ms to an age in wall-ms genuinely needs it.
//
// TURNING THIS BACK ON (=1) restores the old contract for a consumer that integrates on `timestamp`
// and cannot be updated. Do not run mixed: with it off, integrating on the wall stamp over-counts by
// 1/sim_wall_rate_ (~6% here), which is exactly the bug 5968a65 fixed.
#ifndef BRIDGE_WALL_TIME_VELOCITIES          // -DBRIDGE_WALL_TIME_VELOCITIES=n overrides
#define BRIDGE_WALL_TIME_VELOCITIES 0
#endif

// Set to 1 to emit yawrate_diag.csv (sim_t, wall_ms, yaw_raw, wz, vx_world, vy_world) from
// receiving_robotSpeed, for the d(yaw)/dt vs getVelocity()[5] check. Off = zero cost.
#define BRIDGE_YAWRATE_DIAG 0

// Set to 1 to emit wheel_odom_diag.csv from receiving_robotSpeed: the encoder-derived twist and the
// supervisor's, both in the SAME cycle and both per-SIM-second, so the difference between them IS the
// mecanum slip. Everything needed to settle the FK's signs, bound the slip, and check the at-rest
// floor against the isMoving threshold is in that one file. Off = zero cost.
#ifndef BRIDGE_WHEEL_ODOM_DIAG               // -DBRIDGE_WHEEL_ODOM_DIAG=1 turns it on
#define BRIDGE_WHEEL_ODOM_DIAG 0
#endif

// EMA on the encoder-derived linear channels. 1.0 = off, which is how this ships.
//
// There is no measurement justifying a filter yet, and two reasons to expect none is needed. The
// PositionSensors declare no `resolution` in Shadow.proto, so getValue() is the joint angle to double
// precision and differencing two O(1e2) doubles costs ~1e-11 rad/s of numerical noise. And the
// PUBLISHED value is differenced over the whole 100 ms publish interval (see WheelOdometry below),
// not over one 8 ms step, which already averages away the velocity-PID transient that fighting roller
// contact produces. Turn this on only if the at-rest run in the diagnostic shows a floor above the
// 0.001 isMoving threshold, and write the measured number here when you do -- the same discipline
// WHEEL_CENTROID_OFFSET follows above.
#ifndef BRIDGE_WHEEL_ODOM_EMA_ALPHA
#define BRIDGE_WHEEL_ODOM_EMA_ALPHA 1.0
#endif
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
	void Webots2Robocomp_setDoorAngle(std::string DEF, float angle);
	void Webots2Robocomp_setPathToHuman(int humanId, RoboCompGridder::TPath path);
    void Webots2Robocomp_setArmJointsInstant(RoboCompKinovaArm::TJointAngles angles){};
	void Webots2Robocomp_setObjectPose(std::string DEF, RoboCompWebots2Robocomp::ObjectPose pose){};
	void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

	// --- DoorControl: this simulator's implementation of a provider-agnostic door actuator -------
	// The same requests a home-automation bridge or a speech component would serve, so a caller
	// never has to know it is talking to Webots. Underneath it is still the supervisor teleport,
	// but that is this provider's business, not the interface's.
	RoboCompDoorControl::Capabilities DoorControl_getCapabilities();
	RoboCompDoorControl::RequestAck DoorControl_requestDoor(RoboCompDoorControl::DoorRequest req);
	RoboCompDoorControl::RequestStatus DoorControl_getRequestStatus(int requestId);
	void DoorControl_cancelRequest(int requestId);

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
	// DEF -> node cache for setDoorAngle. Replaces the single hard-wired CONTROLLABLE_DOOR: which door
	// a caller means is now its argument, and getFromDef is walked once per DEF (a null is cached too —
	// "no such node in this world" does not change while the world is loaded).
	std::map<std::string, webots::Node*> doorNodes;
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

    // --- Sim-to-wall rate, for publishing velocities in per-WALL-second units ------
    // Webots velocities are per SIMULATION second, but every timestamp this bridge emits is
    // wall-clock, so a consumer that integrates rate * dt(wall) over-counts by exactly the amount
    // the simulation is running behind real time. Measured 2026-08-09: the sim advanced 57.40 s
    // while the wall advanced 61.31 s (wall/sim = 1.068), and room_concept's odometry prior
    // accumulated 6.7% more rotation than the robot actually turned. Integrating the SAME published
    // rate over sim time instead reproduced the node's own yaw to 0.07%, so the value was never
    // wrong -- the units and the timestamps simply disagreed.
    // sim_wall_rate_ = d(sim)/d(wall) over a short trailing window; multiplying the published
    // velocities by it converts them to per-wall-second and makes them consistent with the stamps.
    // 1.0 until enough samples accrue, and while the sim keeps real time it stays 1.0.
    std::deque<std::pair<double, long>> clock_pairs_;   // (sim seconds, wall ms)
    double sim_wall_rate_ = 1.0;
    void   update_sim_wall_rate(double sim_seconds, long wall_ms);

    // --- Wheel odometry: the velocity this bridge PUBLISHES on FullPoseEstimationPub -------------
    // Until now that channel carried robotNode->getVelocity() -- the supervisor's ground truth. It has
    // no wheel slip, and nothing in the sim exercised the path the real robot actually uses: the base
    // controller reads its four wheel speeds and inverts the mecanum kinematics
    // (SVD48VBase/src/specificworker.py:227, `inv_m_wheels @ driver.get_speed()`). Downstream that
    // absence is a documented blocker -- room_concept's se2_preintegration.h:85 cannot identify an
    // odometry scale in simulation because "there is no encoder in the sim loop at all".
    //
    // The four PositionSensors were already resolved and enabled in initialize(); getValue() had
    // simply never been called. So this is the FORWARD kinematics, the exact algebraic inverse of the
    // IK in apply_pending_speed_command() and built from the same WHEEL_RADIUS/LX/LY so the two cannot
    // drift apart:
    //     advz = R/4       * ( w0 + w1 + w2 + w3)
    //     advx = R/4       * (-w0 + w1 + w2 - w3)
    //     rot  = R/(4*c)   * ( w0 - w1 + w2 - w3)      c = LX + LY
    // The IK matrix is column-orthogonal (norms 4, 4, 4c^2), so that least-squares inverse is exact.
    //
    // The GYRO IS DELIBERATELY NOT FUSED IN HERE. FullPoseEstimationPub is the *wheel* channel, the
    // sim's stand-in for SVD48VBase; the IMU is a separate Ice interface the consumer reads and
    // integrates itself. Pre-fusing them in the bridge would leave that consumer no way to weigh the
    // two sources against each other, which is the entire job of the filter component this arrangement
    // stands in for. Consequence worth knowing: `rot` therefore rides on (LX+LY) again, so the LY
    // calibration in commit 5968a65 matters for this channel in a way it would not if the gyro
    // supplied the yaw rate.
    //
    // TWO ANCHORS, and the distinction matters. `pub_*` is differenced over the whole publish interval
    // (100 ms), so the published velocity is the exact MEAN over the interval a consumer will integrate
    // it across -- integrating it reproduces the true wheel displacement. Sampling an 8 ms snapshot
    // every 100 ms instead would alias the roller-contact transient and would NOT integrate correctly.
    // `step_*` is per-step and exists only to feed the diagnostic.
    enum class VelocitySource { Wheels, Supervisor };
    VelocitySource fullpose_velocity_source_ = VelocitySource::Wheels;

    // --- Which running gear this world's robot actually has -------------------------------------
    // Shadow.proto is a four-wheel mecanum; ShadowDiff.proto is the real robot's layout -- two drive
    // wheels on the lateral axis plus front and back casters. Both exist, so the kinematics is chosen
    // at runtime rather than compiled in, and a world swap needs only a config line.
    //
    // The differential constants are the real robot's own, from
    // SVD48VBase/etc/config_diferential: wheelRadius = 100 mm, axesLength = 518 mm. That component
    // uses axesLength as the TRACK (its 2x2 m_wheels is [[-1, L/2], [-1, -L/2]]), so the half-track
    // is 0.259. Using its numbers rather than fitted ones is the whole point: a constant measured
    // against the simulator is a constant that only describes the simulator.
    //
    //   mecanum       advz = R/4     * ( w0 + w1 + w2 + w3)
    //                 advx = R/4     * (-w0 + w1 + w2 - w3)
    //                 rot  = R/(4c)  * ( w0 - w1 + w2 - w3)      c = LX + LY
    //   differential  advz = R/2     * ( wl + wr)
    //                 advx = 0                                   it physically cannot strafe
    //                 rot  = R/(2c)  * ( wr - wl)                c = half-track
    //
    // Both keep the axis -1 0 0 convention, so a positive motor velocity still means forward and the
    // sign handling downstream is identical.
    //
    // MEASURED 2026-08-15, differential base in piso.wbt, 810 publish intervals against the
    // supervisor (which OmniRobot still serves, which is why the comparison is possible at all):
    //     adv   1.000  (n=574)   translation is exact; also confirms R = 0.100
    //     rot   1.082  (n=114 pivots)   the wheels over-report rotation by 8.2%
    //     side  0                a differential base cannot strafe, and the model says so
    // The 8.2% is SCRUB, and it is left in on purpose. A differential base turns by dragging its
    // wheels sideways, so some commanded rotation is always lost; real wheel odometry over-reports
    // rotation for exactly this reason. That error is the thing a wheels+IMU filter exists to
    // correct -- the gyro reads true yaw (measured gyro/sup = 1.065, i.e. 1/k_wall) while the wheels
    // drift. Calibrating half_track_ to the effective 0.280 would make sim odometry unrealistically
    // perfect and remove the very signal the fusion is meant to consume. Do not "fix" it.
    //
    // CONSEQUENCE FOR CONTROL, which is not odometry's problem but is real: the IK uses the same
    // constant, so a commanded rotation under-delivers by about the same 8%. Anything steering
    // open-loop on a commanded omega will fall short; close the loop.
    //
    // For contrast, the same measurement on the MECANUM base in this world read rot 0.845 -- the body
    // turning FASTER than the wheels implied, which is not a thing wheels can do. That was piso.wbt
    // missing the ContactProperties that make mecanum rollers slide (see the note in the world file);
    // without them the rollers grip and the base is a skid-steer wearing mecanum wheels.
    enum class BaseKinematics { Mecanum, Differential };
    BaseKinematics base_kinematics_ = BaseKinematics::Mecanum;
    int    wheel_count_  = 4;
    double wheel_radius_ = WHEEL_RADIUS;
    double half_track_   = LX + LY;      // the c above
    bool   warned_advx_ignored_ = false; // a differential base cannot strafe; say so once, not per call

    struct WheelOdometry
    {
        double step_angle[4]{};             // rad, per-step anchor      (diagnostic only)
        double step_sim_t = 0.0;
        double pub_angle[4]{};              // rad, per-publish anchor   (what gets published)
        double pub_sim_t  = 0.0;
        bool   seeded     = false;          // both anchors hold a real sample
        Eigen::Vector2f v_local{0.f, 0.f};  // per SIM second. (0) = -side, (1) = forward -- the
                                            // shadow_velocity_local convention, see receiving_robotSpeed
        double rot        = 0.0;            // rad/sim-s about local +z, encoders only
        double step_w[4]{};                 // per-wheel rates over one step   (diagnostic only)
        Eigen::Vector2f step_v_local{0.f, 0.f};
        double step_rot   = 0.0;
        double pub_dt     = 0.0;            // s, the interval v_local/rot were differenced over
        bool   valid      = false;          // v_local/rot hold a usable measurement
    };
    WheelOdometry wheel_odo_;               // compute()-thread only; no lock, like clock_pairs_
    bool update_wheel_odometry(double sim_seconds, bool publishing);

    // Publish period for the wheel channel. The IMU stays at the step rate; that rate GAP is what a
    // consumer's high-rate injection propagates across. 100 ms matches the real base
    // (SVD48VBase/etc/config_omnidirectional: Period.Compute = 100).
    int    fullpose_publish_period_ms = 100;
    double next_fullpose_publish_ms   = 0.0;

    // Nominal per-sample variances, SI units squared, from config. The simulated gyro and encoders are
    // EXACT and the accelerometer only quantises at 0.001, so a measured sim covariance would be ~0 --
    // and a zero variance reads downstream as infinite confidence, which wrecks any filter. So publish
    // plausible real-robot figures and let the real IMU and base controller overwrite them once
    // calibrated. (room_config.h already records sigma_v = 3.9e-7 m/sqrt(s) over 24434 parked frames,
    // five orders below what the consumer's motion prior is configured with.)
    // ── Synthetic sensor error (SIMULATION FIDELITY, not a test hook) ────────────────────────────
    // This bridge publishes the supervisor's GROUND TRUTH. Measured in room_concept over 24434 parked
    // frames, the resulting odometry has sigma_v = 3.9e-7 m/sqrt(s) — five to six orders below what
    // every consumer's motion model is configured for. So the motion prior does no work in simulation,
    // and anything tuned against it is tuned against a robot that does not exist.
    //
    // ★ THE WHEELS AND THE GYRO GET SEPARATE ERROR MODELS, AND THAT IS THE POINT. The reason a gyro
    // heading path exists at all is that a gyro does NOT share the wheels' rotation error: a
    // differential base over-reports rotation because it turns by SCRUBBING (the measured wheel/gyro
    // ratio on this robot is ~1.05-1.08). Applying one error to both channels would simulate a gyro
    // that scrubs, make the gyro look useless, and argue against a feature that is genuinely valuable
    // on hardware. So: wheels carry a SCALE plus encoder noise; the gyro carries its own small noise
    // and a slow BIAS, and no scale at all.
    //
    // sigma_* are DENSITIES (m/sqrt(s), rad/sqrt(s)): the rate is perturbed by sigma/sqrt(dt) so the
    // accumulated increment has variance sigma^2*dt. That is what makes one constant mean the same
    // thing on the 10 Hz wheel channel and the 125 Hz IMU channel.
    // ⚠ All zero (the default) is inert and the bridge publishes ground truth exactly as before.
    bool   sensor_noise_enabled_   = false;
    // ★ SLIP COEFFICIENTS per unit speed since 2026-08-29, not flat densities. See the note at the
    //   injection site: an encoder reporting zero counts is not a noisy reading of zero, it is the
    //   ABSENCE of counts, and a flat density injects motion into a parked robot that the consumer
    //   then integrates. sigma(v) = hypot(floor, k_slip * |v|).
    double wheel_sigma_v_floor_    = 0.0005; // m/sqrt(s)   — sub-count residual at standstill
    double wheel_sigma_w_floor_    = 0.0010; // rad/sqrt(s) — same, yaw
    double wheel_sigma_v_eff_      = 0.0;    // the sigma ACTUALLY used for this sample
    double wheel_sigma_w_eff_      = 0.0;
    double wheel_sigma_v_          = 0.0;   // m/sqrt(s)   — encoder velocity noise density
    double wheel_sigma_w_          = 0.0;   // rad/sqrt(s) — wheel-derived yaw-rate noise density
    double wheel_scale_v_          = 0.0;   // fraction: reported = true*(1+s), constant for the run
    double wheel_scale_w_          = 0.0;   // fraction — the scrubbing error, ~+0.06 on this base
    double gyro_sigma_             = 0.0;   // rad/sqrt(s) — gyro noise density, INDEPENDENT of the wheels
    double gyro_bias_              = 0.0;   // rad/s       — constant gyro offset, the error a gyro DOES have
    double nominal_gyro_var_ = 1e-5;        // (rad/s)^2
    double nominal_acc_var_  = 1e-3;        // (m/s^2)^2
    double nominal_vel_var_  = 1e-4;        // (m/s)^2
    double nominal_rot_var_  = 1e-4;        // (rad/s)^2

    // Acquisition instant of a device's current sample, on the sim clock. Webots refreshes a device on
    // steps that are multiples of its sampling period, so the sample in hand was taken at the last such
    // multiple -- NOT at the loop time, which is what every payload used to be stamped with even though
    // a lidar frame can be a full sensor period old. Returns ms.
    static double acquisition_sim_ms(double sim_now_ms, int device_period_ms);
    // ... and its wall-clock equivalent, for the existing `timestamp` field.
    long acquisition_wall_ms(long loop_wall_ms, double sim_now_ms, int device_period_ms) const;

#if BRIDGE_WHEEL_ODOM_DIAG
    // What the wheels were last actually told, for the diagnostic's cmd_* columns.
    struct AppliedCommand { double advx = 0.0, advz = 0.0, rot = 0.0; };
    AppliedCommand last_applied_cmd_;       // compute()-thread only
    void log_wheel_odom_diag(double sim_now_ms, long wall_ms, bool publishing,
                             const Eigen::Vector2f &sup_v_local_sim, double sup_wz,
                             const double *orientation_matrix);
#endif

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

    // --- DoorControl state -------------------------------------------------------
    // The doors this provider advertises are the hinge DEFs named in config
    // (DoorControl.Doors). Their poses are refreshed from the Webots thread like every
    // other snapshot here; the servants only ever read the cache under the mutex.
    // The caller matches these poses against its own fitted apertures — this provider
    // has no idea what a "room" is, which is exactly what makes it swappable for a
    // home-automation bridge or a text-to-speech component.
    std::mutex door_control_mutex;
    RoboCompDoorControl::DoorRefList door_control_doors;   // advertised, pose in mm
    std::map<int, RoboCompDoorControl::RequestStatus> door_control_requests;
    int door_control_next_id = 1;
    std::vector<std::string> door_control_defs;            // from config, compute()-thread only
    void update_door_control();

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
    // The STEP period, kept because the IMU samples every step while `sensor_period` is the (possibly
    // slower) heavy-sensor rate. Used to turn a gyro noise DENSITY into a per-sample perturbation:
    // getting that interval wrong would make the same sigma mean different drift on different
    // channels, which is exactly the rate-dependence the density form exists to remove.
    int imu_step_period_ms_ = 0;
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

	void setDoorAperture(const std::string &DEF, float _aperture);

signals:
        //void customSignal();
};

#endif
