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
#include "specificworker.h"

#pragma region Robocomp Methods

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;

    if(robot)
        delete robot;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
    // Save locale setting
    const std::string oldLocale=std::setlocale(LC_NUMERIC,nullptr);
    // Force '.' as the radix point. If you comment this out,
    // you'll get output similar to the OP's GUI mode sample
    std::setlocale(LC_NUMERIC,"C");
    try
    {
        pars.delay = params.at("delay").value == "true" or (params.at("delay").value == "True");
    }catch (const std::exception &e)
    {std::cout <<"Error reading the config \n" << e.what() << std::endl << std::flush; }

    // Restore locale setting
    std::setlocale(LC_NUMERIC,oldLocale.c_str());
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
        robot = new webots::Supervisor();

        // Inicializa los motores y los sensores de posición.
        const char *motorNames[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
        //const char *sensorNames[4] = {"wheel1sensor", "wheel2sensor", "wheel3sensor", "wheel4sensor"};

        // Inicializa los sensores soportados.
        lidar_helios = robot->getLidar("helios");
        lidar_pearl = robot->getLidar("bpearl");
        //camera = robot->getCamera("camera");
        //range_finder = robot->getRangeFinder("range-finder");
        camera360_1 = robot->getCamera("camera_360_1");
        camera360_2 = robot->getCamera("camera_360_2");

        this->Period = 1;

        // Activa los componentes en la simulación si los detecta.
        if(lidar_helios) lidar_helios->enable(this->Period);
        if(lidar_pearl) lidar_pearl->enable(this->Period);
        if(camera) camera->enable(this->Period);
        if(range_finder) range_finder->enable(this->Period);
        if(camera360_1 && camera360_2){
            camera360_1->enable(this->Period);
            camera360_2->enable(this->Period);
        }
        for (int i = 0; i < 4; i++)
        {
            motors[i] = robot->getMotor(motorNames[i]);
            ps[i] = motors[i]->getPositionSensor();
            ps[i]->enable(this->Period);
            motors[i]->setPosition(INFINITY); // Modo de velocidad.
            motors[i]->setVelocity(0);
        }

        //Insert in the config?
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
//    auto now = std::chrono::system_clock::now();

    // Getting the data from simulation.
    if(lidar_helios) receiving_lidarData(lidar_helios, double_buffer_helios,  helios_delay_queue);
    if(lidar_pearl) receiving_lidarData(lidar_pearl, double_buffer_pearl, pearl_delay_queue);
    if(camera) receiving_cameraRGBData(camera);
    if(range_finder) receiving_depthImageData(range_finder);
    if(camera360_1 && camera360_2) receiving_camera360Data(camera360_1, camera360_2);
//    robot->step(this->Period);
    robot->step(1);
//    std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count() << std::endl;

    fps.print("FPS:");
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

#pragma endregion Robocomp Methods

#pragma region Data-Catching Methods

void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2)
{
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Timestamp calculation
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    newImage360.alivetime = millis;

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();

    // Establecer el periodo de refresco de la imagen en milisegundos.
//    newImage360.period = this->Period;

    // Establecer el periodo real del compute de refresco de la imagen en milisegundos.
    newImage360.period = fps.get_period();

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();
    cv::Mat img_1 = cv::Mat(cv::Size(_camera1->getWidth(), _camera1->getHeight()), CV_8UC4);
    cv::Mat img_2 = cv::Mat(cv::Size(_camera2->getWidth(), _camera2->getHeight()), CV_8UC4);
    img_1.data = (uchar *)webotsImageData1;
    cv::cvtColor(img_1, img_1, cv::COLOR_RGBA2RGB);
    img_2.data = (uchar *)webotsImageData2;
    cv::cvtColor(img_2, img_2, cv::COLOR_RGBA2RGB);
    cv::Mat img_final = cv::Mat(cv::Size(_camera1->getWidth()*2, _camera1->getHeight()), CV_8UC3);
    img_1.copyTo(img_final(cv::Rect(0, 0, _camera1->getWidth(), _camera1->getHeight())));
    img_2.copyTo(img_final(cv::Rect(_camera1->getWidth(), 0, _camera1->getWidth(), _camera2->getHeight())));

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

void SpecificWorker::receiving_lidarData(webots::Lidar* _lidar, DoubleBuffer<RoboCompLidar3D::TData, RoboCompLidar3D::TData> &_lidar3dData, FixedSizeDeque<RoboCompLidar3D::TData>& delay_queue)
{
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double minRange = _lidar->getMinRange();
    double maxRange = _lidar->getMaxRange();
    double fov = _lidar->getFov();
    double verticalFov = _lidar->getVerticalFov();

    // Timestamp calculation
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    // Configuration settings
    RoboCompLaser::TLaserData newLaserData;
    RoboCompLaser::LaserConfData newLaserConfData;
    RoboCompLidar3D::TData newLidar3dData;

    // General Lidar values
    newLidar3dData.timestamp = millis;
    newLidar3dData.period = fps.get_period();
    newLaserConfData.maxDegrees = fov;
    newLaserConfData.maxRange = maxRange;
    newLaserConfData.minRange = minRange;
    newLaserConfData.angleRes = fov / horizontalResolution;

    //std::cout << "horizontal resolution: " << horizontalResolution << " vertical resolution: " << verticalResolution << " fov: " << fov << " vertical fov: " << verticalFov << std::endl;

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    for (int j = 0; j < verticalResolution; ++j) {
        for (int i = 0; i < horizontalResolution; ++i) {
            int index = j * horizontalResolution + i;

            //distance meters to millimeters
            const float distance = rangeImage[index]; //Meters

            //TODO rotacion del eje y con el M_PI, solucionar
            float horizontalAngle = M_PI - i * newLaserConfData.angleRes - fov / 2;
            float verticalAngle = M_PI + j * (verticalFov / verticalResolution) - verticalFov / 2;

            //Calculate Cartesian co-ordinates and rectify axis positions
            Eigen::Vector3f lidar_point(
                distance * cos(horizontalAngle) * cos(verticalAngle),
                distance * sin(horizontalAngle) * cos(verticalAngle),
                distance * sin(verticalAngle));

            if (not (std::isinf(lidar_point.x()) or std::isinf(lidar_point.y()) or std::isinf(lidar_point.z())))
            {
                RoboCompLidar3D::TPoint point;

                point.x = lidar_point.x();
                point.y = lidar_point.y();
                point.z = lidar_point.z();

                point.r = lidar_point.norm();  // distancia radial
                point.phi = horizontalAngle;  // ángulo horizontal // -x para hacer [PI, -PI] y no [-PI, PI]
                point.theta = verticalAngle;  // ángulo vertical
                point.distance2d = std::hypot(lidar_point.x(),lidar_point.y());  // distancia en el plano xy

                RoboCompLaser::TData data;
                data.angle = point.phi;
                data.dist = point.distance2d;

                newLidar3dData.points.push_back(point);
                newLaserData.push_back(data);
            }
        }
    }
    //Points order to angles
    std::ranges::sort(newLidar3dData.points, {}, &RoboCompLidar3D::TPoint::phi);

    laserData = newLaserData;
    laserDataConf = newLaserConfData;

    //Is it necessary to use two lidar queues? One for each lidaR?
    if(pars.delay)
        delay_queue.push(newLidar3dData);

    _lidar3dData.put(std::move(newLidar3dData));
}
void SpecificWorker::receiving_cameraRGBData(webots::Camera* _camera){
    RoboCompCameraRGBDSimple::TImage newImage;

    // Se establece el periodo de refresco de la imagen en milisegundos.
//    newImage.period = this->Period;
    newImage.period = fps.get_period();

    // Timestamp calculation
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    newImage.alivetime = millis;

    // Obtener la resolución de la imagen.
    newImage.width = _camera->getWidth();
    newImage.height = _camera->getHeight();

    const unsigned char* webotsImageData = _camera->getImage();

    // Crear un vector para la nueva imagen RGB.
    std::vector<unsigned char> rgbImage;
    rgbImage.reserve(3 * newImage.width * newImage.height);  // Reservar espacio para RGB

    for (int y = 0; y < newImage.height; y++)
    {
        for (int x = 0; x < newImage.width; x++)
        {
            // Extraer cada canal por separado
            unsigned char r = _camera->imageGetRed(webotsImageData, newImage.width, x, y);
            unsigned char g = _camera->imageGetGreen(webotsImageData, newImage.width, x, y);
            unsigned char b = _camera->imageGetBlue(webotsImageData, newImage.width, x, y);

            // Añadir los canales al vector BGR final.
            rgbImage.push_back(b);
            rgbImage.push_back(g);
            rgbImage.push_back(r);
        }
    }

    // Asignar la imagen RGB al tipo TImage de Robocomp
    newImage.image = rgbImage;
    newImage.compressed = false;

    // Asignamos el resultado final al atributo de clase
    this->cameraImage = newImage;
}
void SpecificWorker::receiving_depthImageData(webots::RangeFinder* _rangeFinder){
    RoboCompCameraRGBDSimple::TDepth newDepthImage;

    // Se establece el periodo de refresco de la imagen en milisegundos.
    newDepthImage.period = fps.get_period();

    // Obtener la resolución de la imagen de profundidad.
    newDepthImage.width = _rangeFinder->getWidth();
    newDepthImage.height = _rangeFinder->getHeight();
    newDepthImage.depthFactor = _rangeFinder->getMaxRange();
    newDepthImage.compressed = false;

    // Obtener la imagen de profundidad
    const float* webotsDepthData = _rangeFinder->getRangeImage();

    // Accedemos a cada depth value y le aplicamos un factor de escala.
    const int imageElementCount = newDepthImage.width * newDepthImage.height;

    for(int i= 0 ; i<imageElementCount; i++){

        // Este es el factor de escala a aplicar.
        float scaledValue = webotsDepthData[i] * 10;

        // Convertimos de float a array de bytes.
        unsigned char singleElement[sizeof(float)];
        memcpy(singleElement, &scaledValue, sizeof(float));

        for(uint j=0; j<sizeof(float); j++){
            newDepthImage.depth.emplace_back(singleElement[j]);
        }
    }

    // Asignamos el resultado final al atributo de clase
    this->depthImage = newDepthImage;
}

#pragma endregion Data-Catching Methods

#pragma region CameraRGBDSimple

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    RoboCompCameraRGBDSimple::TRGBD newRGBD;

    newRGBD.image = this->cameraImage;
    newRGBD.depth = this->depthImage;
    // TODO: Que devuelva tambien la nube de puntos.

    return newRGBD;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    return this->depthImage;
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    return this->cameraImage;
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
    printNotImplementedWarningMessage("CameraRGBDSimple_getPoints");
    return RoboCompCameraRGBDSimple::TPoints();
}

#pragma endregion CamerRGBDSimple

#pragma region Lidar

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    return laserData;
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    return laserDataConf;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    return laserData;
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationDegreeFactor)
{
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
        cout << "Getting data from a not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << endl;
        return RoboCompLidar3D::TData();
    }
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance)
{
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
        cout << "Getting data from a not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << endl;
        return RoboCompLidar3D::TData();
    }
}

RoboCompLidar3D::TDataImage SpecificWorker::Lidar3D_getLidarDataArrayProyectedInImage(std::string name)
{
    printNotImplementedWarningMessage("Lidar3D_getLidarDataArrayProyectedInImage");
    return RoboCompLidar3D::TDataImage();
}

#pragma endregion Lidar

#pragma region Camera360

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    if(pars.delay)
    {
        if(camera_queue.full())
            return camera_queue.back();
    }

    return double_buffer_360.get_idemp();
}

#pragma endregion Camera360


#pragma region OmniRobot

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    printNotImplementedWarningMessage("OmniRobot_getBasePose");
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    webots::Node *robotNode = robot->getFromDef("shadow");

    state.x = robotNode->getField("translation")->getSFVec3f()[0];
    state.z = robotNode->getField("translation")->getSFVec3f()[1];
    state.alpha = robotNode->getField("rotation")->getSFRotation()[0];
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    printNotImplementedWarningMessage("OmniRobot_resetOdometer");
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometer");
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    double speeds[4];
    speeds[0] = 1.0 / WHEEL_RADIUS * (advx + advz + (LX + LY) * rot);
    speeds[1] = 1.0 / WHEEL_RADIUS * (advx - advz - (LX + LY) * rot);
    speeds[2] = 1.0 / WHEEL_RADIUS * (advx - advz + (LX + LY) * rot);
    speeds[3] = 1.0 / WHEEL_RADIUS * (advx + advz - (LX + LY) * rot);
    printf("Speeds: vx=%.2f[m/s] vy=%.2f[m/s] ω=%.2f[rad/s]\n", advx, advz, rot);
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(speeds[i]);
    }
}

void SpecificWorker::OmniRobot_stopBase()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i]->setVelocity(0);
    }
}

#pragma endregion OmniRobot

#pragma region JoystickAdapter

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    // Declaration of the structure to be filled
    float advx=0, advz=0, rot=0;

    /*
    // Iterate through the list of buttons in the data structure
    for (RoboCompJoystickAdapter::ButtonParams button : data.buttons) {
        // Currently does nothing with the buttons
    }
    */

    // Iterate through the list of axes in the data structure
    for (RoboCompJoystickAdapter::AxisParams axis : data.axes){
        // Process the axis according to its name
        if(axis.name == "rotate") {
            rot = axis.value;
        }
        else if (axis.name == "advance") {
            advx = axis.value;
        }
        else if (axis.name == "side") {
            advz = axis.value;
        }
        else {
            cout << "[ JoystickAdapter ] Warning: Using a non-defined axes (" << axis.name << ")." << endl;
        }
    }

    OmniRobot_setSpeedBase(advx, advz, rot);
}

#pragma endregion JoystickAdapter

RoboCompLidar3D::TData SpecificWorker::filterLidarData(const RoboCompLidar3D::TData _lidar3dData, float _start, float _len, int _decimationDegreeFactor){

    RoboCompLidar3D::TData buffer = _lidar3dData;
// Check for nominal conditions
    if(_len == 360  and _decimationDegreeFactor == 1)
        return buffer;

    RoboCompLidar3D::TPoints filtered_points; 
    //Get all LiDAR
    if (_len == 360)
        filtered_points = std::move(buffer.points);
    //Cut range LiDAR
    else{
        //Start and end angles
        auto rad_start = _start;
        auto rad_end = _start + _len;
        
        //Start Iterator, this is the end if there are surpluses, otherwise it will be modified by the defined end.
        auto it_begin = std::find_if(buffer.points.begin(), buffer.points.end(),
                    [_start=rad_start](const RoboCompLidar3D::TPoint& point) 
                    {return _start < point.phi;});
        //End Iterator
        auto it_end = buffer.points.end();
        //The clipping exceeds 2pi, we assign the excess to the result
        if (rad_end > M_PI)
            filtered_points.assign(std::make_move_iterator(buffer.points.begin()), std::make_move_iterator(std::find_if(buffer.points.begin(), buffer.points.end(),
                        [_end=rad_end - 2*M_PI](const RoboCompLidar3D::TPoint& point) 
                        {return _end < point.phi;})));
        else 
            it_end = std::find_if(it_begin, buffer.points.end(),
                        [_end=rad_end](const RoboCompLidar3D::TPoint& point)
                        {return _end < point.phi;});
        //we insert the cut with 2PI limit
        filtered_points.insert(filtered_points.end(), std::make_move_iterator(it_begin), std::make_move_iterator(it_end));
        #if DEBUG
            std::cout << "Time prepare lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
        #endif
    }
    //
    if (_decimationDegreeFactor == 1)
        return RoboCompLidar3D::TData {filtered_points, buffer.period, buffer.timestamp};
    #if DEBUG   
        cstart = std::chrono::high_resolution_clock::now();
    #endif

    //Decimal factor calculation
    float rad_factor = qDegreesToRadians((float)_decimationDegreeFactor);
    float tolerance = qDegreesToRadians(0.5);

    //We remove the points that are of no interest 
    filtered_points.erase(std::remove_if(filtered_points.begin(), filtered_points.end(),
            [rad_factor, tolerance](const RoboCompLidar3D::TPoint& point) 
            {float remainder = abs(fmod(point.phi, rad_factor));
                return !(remainder <= tolerance || remainder >= rad_factor - tolerance);
            }), filtered_points.end());
    #if DEBUG
        std::cout << "Time for cut lidar: " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - cstart).count() << " microseconds" << std::endl<<std::flush;
    #endif
    return RoboCompLidar3D::TData {filtered_points, buffer.period, buffer.timestamp};
}
void SpecificWorker::printNotImplementedWarningMessage(string functionName)
{
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
}


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
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData

/**************************************/
// From the RoboCompLidar3D you can use this types:
// RoboCompLidar3D::TPoint
// RoboCompLidar3D::TData

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

