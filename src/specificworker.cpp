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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

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
		timer.start(Period);
	}

	robot = new webots::Robot();

    // Inicializa los motores y los sensores de posición.
    const char *motorNames[4] = {"wheel1", "wheel2", "wheel3", "wheel4"};
    const char *sensorNames[4] = {"wheel1sensor", "wheel2sensor", "wheel3sensor", "wheel4sensor"};

    // Inicializa los sensores soportados.
    lidar_helios = robot->getLidar("helios");
    lidar_pearl = robot->getLidar("pearl");
    camera = robot->getCamera("camera");
    range_finder = robot->getRangeFinder("range-finder");
    camera360_1 = robot->getCamera("camera_360_1");
    camera360_2 = robot->getCamera("camera_360_2");

    // Activa los componentes en la simulación si los detecta.
    if(lidar_helios) lidar_helios->enable(TIME_STEP);
    if(lidar_pearl) lidar_pearl->enable(TIME_STEP);
    if(camera) camera->enable(TIME_STEP);
    if(range_finder) range_finder->enable(TIME_STEP);
    if(camera360_1 && camera360_2){
        camera360_1->enable(TIME_STEP);
        camera360_2->enable(TIME_STEP);
    }
    for (int i = 0; i < 4; i++) {
        motors[i] = robot->getMotor(motorNames[i]);
        ps[i] = motors[i]->getPositionSensor();
        ps[i]->enable(TIME_STEP);
        motors[i]->setPosition(INFINITY); // Modo de velocidad.
        motors[i]->setVelocity(0);
    }

    // Realiza la primera iteración de simulación.
    robot->step(100);
}

void SpecificWorker::compute()
{
    // Getting the data from simulation.
    if(lidar_helios) receiving_lidarData(lidar_helios, lidar3dData_helios);
    if(lidar_pearl) receiving_lidarData(lidar_pearl, lidar3dData_pearl);
    if(camera) receiving_cameraRGBData(camera);
    if(range_finder) receiving_depthImageData(range_finder);
    if(camera360_1 && camera360_2) receiving_camera360Data(camera360_1, camera360_2);


    // Setting the simulator timestep.
    robot->step(100);
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

#pragma endregion Robocomp Methods

#pragma region Data-Catching Methods

void SpecificWorker::receiving_camera360Data(webots::Camera* _camera1, webots::Camera* _camera2){
    RoboCompCamera360RGB::TImage newImage360;

    // Aseguramos de que ambas cámaras tienen la misma resolución, de lo contrario, deberás manejar las diferencias.
    if (_camera1->getWidth() != _camera2->getWidth() || _camera1->getHeight() != _camera2->getHeight())
    {
        std::cerr << "Error: Cameras with different resolutions." << std::endl;
        return;
    }

    // Establecer el periodo de refresco de la imagen en milisegundos.
    newImage360.period = 30;

    // La resolución de la nueva imagen será el doble en el ancho ya que estamos combinando las dos imágenes.
    newImage360.width = 2 * _camera1->getWidth();
    newImage360.height = _camera1->getHeight();

    const unsigned char* webotsImageData1 = _camera1->getImage();
    const unsigned char* webotsImageData2 = _camera2->getImage();

    // Crear un vector para la nueva imagen RGB 360.
    std::vector<unsigned char> rgbImage360;
    rgbImage360.reserve(3 * newImage360.width * newImage360.height);  // Reservar espacio para RGB

    unsigned char r;
    unsigned char g;
    unsigned char b;

    // Combinamos los pixeles de ambas cámaras en la nueva imagen.
    for (int y = 0; y < _camera1->getHeight(); y++)
    {
        for (int x = 0; x < _camera1->getWidth(); x++)
        {
            r = _camera1->imageGetRed(webotsImageData1, _camera1->getWidth(), x, y);
            g = _camera1->imageGetGreen(webotsImageData1, _camera1->getWidth(), x, y);
            b = _camera1->imageGetBlue(webotsImageData1, _camera1->getWidth(), x, y);

            rgbImage360.push_back(b);
            rgbImage360.push_back(g);
            rgbImage360.push_back(r);
        }

        for (int x = 0; x < _camera2->getWidth(); x++)
        {
            r = _camera2->imageGetRed(webotsImageData2, _camera2->getWidth(), x, y);
            g = _camera2->imageGetGreen(webotsImageData2, _camera2->getWidth(), x, y);
            b = _camera2->imageGetBlue(webotsImageData2, _camera2->getWidth(), x, y);

            rgbImage360.push_back(b);
            rgbImage360.push_back(g);
            rgbImage360.push_back(r);
        }
    }

    // Asignar la imagen RGB 360 al tipo TImage de Robocomp
    newImage360.image = rgbImage360;
    newImage360.compressed = false;

    // Asignamos el resultado final al atributo de clase (si tienes uno).
    this->camera360Image = newImage360;
}

void SpecificWorker::receiving_lidarData(webots::Lidar* _lidar, RoboCompLidar3D::TData &_lidar3dData){
    if (!_lidar) { std::cout << "No lidar available." << std::endl; return; }

    const float *rangeImage = _lidar->getRangeImage();
    int horizontalResolution = _lidar->getHorizontalResolution();
    int verticalResolution = _lidar->getNumberOfLayers();
    double minRange = _lidar->getMinRange();
    double maxRange = _lidar->getMaxRange();
    double fov = _lidar->getFov();
    double verticalFov = _lidar->getVerticalFov();

    RoboCompLaser::TLaserData newLaserData;
    RoboCompLaser::LaserConfData newLaserConfData;
    RoboCompLidar3D::TData newLidar3dData;

    newLaserConfData.maxDegrees = fov;
    newLaserConfData.maxRange = maxRange;
    newLaserConfData.minRange = minRange;
    newLaserConfData.angleRes = fov / horizontalResolution;

    //std::cout << "horizontal resolution: " << horizontalResolution << " vertical resolution: " << verticalResolution << " fov: " << fov << " vertical fov: " << verticalFov << std::endl;

    if(!rangeImage) { std::cout << "Lidar data empty." << std::endl; return; }

    for (int j = 0; j < verticalResolution; ++j) {
        for (int i = 0; i < horizontalResolution; ++i) {
            int index = j * horizontalResolution + i;

            const float distance = rangeImage[index];

            float horizontalAngle = i * newLaserConfData.angleRes - fov / 2;
            float verticalAngle = j * (verticalFov / verticalResolution) - verticalFov / 2;

            RoboCompLaser::TData data;
            data.angle = horizontalAngle;
            data.dist = distance;

            RoboCompLidar3D::TPoint point;
            point.x = distance * cos(horizontalAngle) * cos(verticalAngle);
            point.y = distance * sin(horizontalAngle) * cos(verticalAngle);
            point.z = distance * sin(verticalAngle);
            point.phi = horizontalAngle;  // ángulo horizontal
            point.theta = verticalAngle;  // ángulo vertical
            point.r = distance;  // distancia radial
            point.distance2d = sqrt(point.x * point.x + point.y * point.y);  // distancia en el plano xy

            // std::cout << "X: " << point.x << " Y: " << point.y << " Z: " << point.z << std::endl;

            newLidar3dData.points.push_back(point);
            newLaserData.push_back(data);
        }
    }

    laserData = newLaserData;
    laserDataConf = newLaserConfData;
    _lidar3dData = newLidar3dData;
}

void SpecificWorker::receiving_cameraRGBData(webots::Camera* _camera){
    RoboCompCameraRGBDSimple::TImage newImage;

    // Se establece el periodo de refresco de la imagen en milisegundos.
    newImage.period = TIME_STEP;

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
    newDepthImage.period = TIME_STEP;

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

        for(int j=0; j<sizeof(float); j++){
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

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarData(std::string name, float start, float len, int decimationfactor)
{
    if(name == "helios") {
        return filterLidarData(lidar3dData_helios, start, len, decimationfactor);
    }
    else if(name == "pearl")
        return filterLidarData(lidar3dData_pearl, start, len, decimationfactor);
    else{
        cout << "Getting data from an not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << endl;

        RoboCompLidar3D::TData emptyData;
        return emptyData;
    }
}

RoboCompLidar3D::TData SpecificWorker::Lidar3D_getLidarDataWithThreshold2d(std::string name, float distance)
{
    if(name == "helios") {
        return lidar3dData_helios;
    }
    else if(name == "pearl")
        return lidar3dData_pearl;
    else{
        cout << "Getting data with threshold from an not implemented lidar (" << name << "). Try 'helios' or 'pearl' instead." << endl;

        RoboCompLidar3D::TData emptyData;
        return emptyData;
    }
}

#pragma endregion Lidar

#pragma region Camera360

RoboCompCamera360RGB::TImage SpecificWorker::Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)
{
    return this->camera360Image;
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
    printNotImplementedWarningMessage("OmniRobot_getBaseState");
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
    speeds[0] = 1 / WHEEL_RADIUS * (advx + advz + (LX + LY) * rot);
    speeds[1] = 1 / WHEEL_RADIUS * (advx - advz - (LX + LY) * rot);
    speeds[2] = 1 / WHEEL_RADIUS * (advx - advz + (LX + LY) * rot);
    speeds[3] = 1 / WHEEL_RADIUS * (advx + advz - (LX + LY) * rot);
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
    float advx, advz, rot;

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

    // Stablish new velocities through OmniRobot interfaces
    OmniRobot_setSpeedBase(advx, advz, rot);
}

#pragma endregion JoystickAdapter

RoboCompLidar3D::TData SpecificWorker::filterLidarData(RoboCompLidar3D::TData _lidar3dData, float _start, float _len, int _decimationfactor){

    RoboCompLidar3D::TData filteredData;

    double startRadians = _start * M_PI / 180.0; // Convert to radians
    double lenRadians = _len * M_PI / 180.0; // Convert to radians
    double endRadians = startRadians + lenRadians; // Precompute end angle

    int counter = 0;
    int decimationCounter = 0; // Use a separate counter for decimation

    // Reserve space for points. If decimation is 1, at max we could have same number of points
    if (_decimationfactor == 1)
    {
        filteredData.points.reserve(_lidar3dData.points.size());
    }

    for (int i = 0; i < _lidar3dData.points.size(); i++)
    {
        double angle = atan2(_lidar3dData.points[i].y, _lidar3dData.points[i].x) + M_PI; // Calculate angle in radians
        if (angle >= startRadians && angle <= endRadians)
        {
            if (decimationCounter == 0) // Add decimation factor
            {
                filteredData.points.push_back(_lidar3dData.points[i]);
                //cout << "X: " << _lidar3dData.points[i].x << " Y: " << _lidar3dData.points[i].y << " Z: " << _lidar3dData.points[i].z << " " << endl;
            }
            counter++;
            decimationCounter++;
            if (decimationCounter == _decimationfactor)
            {
                decimationCounter = 0; // Reset decimation counter
            }
        }
    }

    return filteredData;
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

