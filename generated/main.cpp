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


/** \mainpage RoboComp::Webots2Robocomp
 *
 * \section intro_sec Introduction
 *
 * The Webots2Robocomp component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd Webots2Robocomp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/Webots2Robocomp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <ConfigLoader/ConfigLoader.h>

#include <sigwatch/sigwatch.h>

#include "genericworker.h"
#include "../src/specificworker.h"

#include <camera360rgbI.h>
#include <camerargbdsimpleI.h>
#include <imuI.h>
#include <laserI.h>
#include <lidar3dI.h>
#include <omnirobotI.h>
#include <visualelementsI.h>
#include <webots2robocompI.h>
#include <joystickadapterI.h>

#include <Camera360RGB.h>
#include <GenericBase.h>
#include <GenericBase.h>
#include <Gridder.h>
#include <Person.h>

//#define USE_QTGUI

#define PROGRAM_NAME    "Webots2Robocomp"
#define SERVER_FULL_NAME   "RoboComp Webots2Robocomp::Webots2Robocomp"


class Webots2Robocomp : public Ice::Application
{
public:
	Webots2Robocomp (QString configFile, QString prfx, bool startup_check) { 
		this->configFile = configFile.toStdString();
		this->prefix = prfx.toStdString();
		this->startup_check_flag=startup_check; 
		}
private:
	void initialize();
	std::string prefix, configFile;
	ConfigLoader configLoader;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::Webots2Robocomp::initialize()
{
    this->configLoader.load(this->configFile);
	this->configLoader.printConfig();
}

int ::Webots2Robocomp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;


	std::string proxy, tmp;
	initialize();

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->stringToProxy(configLoader.get<std::string>("TopicManager.Proxy")));
		if (!topicManager)
		{
		    std::cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<std::endl;
		    std::cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<std::endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << std::endl;
		return EXIT_FAILURE;
	}

	tprx = std::tuple<>();
	SpecificWorker *worker = new SpecificWorker(this->configLoader, tprx, startup_check_flag);
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));

	try
	{

		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("Camera360RGB.Endpoints");
		    Ice::ObjectAdapterPtr adapterCamera360RGB = communicator()->createObjectAdapterWithEndpoints("Camera360RGB", tmp);
			auto camera360rgb = std::make_shared<Camera360RGBI>(worker);
			adapterCamera360RGB->add(camera360rgb, Ice::stringToIdentity("camera360rgb"));
			adapterCamera360RGB->activate();
			std::cout << "[" << PROGRAM_NAME << "]: Camera360RGB adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Camera360RGB\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("CameraRGBDSimple.Endpoints");
		    Ice::ObjectAdapterPtr adapterCameraRGBDSimple = communicator()->createObjectAdapterWithEndpoints("CameraRGBDSimple", tmp);
			auto camerargbdsimple = std::make_shared<CameraRGBDSimpleI>(worker);
			adapterCameraRGBDSimple->add(camerargbdsimple, Ice::stringToIdentity("camerargbdsimple"));
			adapterCameraRGBDSimple->activate();
			std::cout << "[" << PROGRAM_NAME << "]: CameraRGBDSimple adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for CameraRGBDSimple\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("IMU.Endpoints");
		    Ice::ObjectAdapterPtr adapterIMU = communicator()->createObjectAdapterWithEndpoints("IMU", tmp);
			auto imu = std::make_shared<IMUI>(worker);
			adapterIMU->add(imu, Ice::stringToIdentity("imu"));
			adapterIMU->activate();
			std::cout << "[" << PROGRAM_NAME << "]: IMU adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for IMU\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("Laser.Endpoints");
		    Ice::ObjectAdapterPtr adapterLaser = communicator()->createObjectAdapterWithEndpoints("Laser", tmp);
			auto laser = std::make_shared<LaserI>(worker);
			adapterLaser->add(laser, Ice::stringToIdentity("laser"));
			adapterLaser->activate();
			std::cout << "[" << PROGRAM_NAME << "]: Laser adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Laser\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("Lidar3D.Endpoints");
		    Ice::ObjectAdapterPtr adapterLidar3D = communicator()->createObjectAdapterWithEndpoints("Lidar3D", tmp);
			auto lidar3d = std::make_shared<Lidar3DI>(worker);
			adapterLidar3D->add(lidar3d, Ice::stringToIdentity("lidar3d"));
			adapterLidar3D->activate();
			std::cout << "[" << PROGRAM_NAME << "]: Lidar3D adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Lidar3D\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("OmniRobot.Endpoints");
		    Ice::ObjectAdapterPtr adapterOmniRobot = communicator()->createObjectAdapterWithEndpoints("OmniRobot", tmp);
			auto omnirobot = std::make_shared<OmniRobotI>(worker);
			adapterOmniRobot->add(omnirobot, Ice::stringToIdentity("omnirobot"));
			adapterOmniRobot->activate();
			std::cout << "[" << PROGRAM_NAME << "]: OmniRobot adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for OmniRobot\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("VisualElements.Endpoints");
		    Ice::ObjectAdapterPtr adapterVisualElements = communicator()->createObjectAdapterWithEndpoints("VisualElements", tmp);
			auto visualelements = std::make_shared<VisualElementsI>(worker);
			adapterVisualElements->add(visualelements, Ice::stringToIdentity("visualelements"));
			adapterVisualElements->activate();
			std::cout << "[" << PROGRAM_NAME << "]: VisualElements adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for VisualElements\n";
		}


		try
		{
			// Server adapter creation and publication
		    tmp = configLoader.get<std::string>("Webots2Robocomp.Endpoints");
		    Ice::ObjectAdapterPtr adapterWebots2Robocomp = communicator()->createObjectAdapterWithEndpoints("Webots2Robocomp", tmp);
			auto webots2robocomp = std::make_shared<Webots2RobocompI>(worker);
			adapterWebots2Robocomp->add(webots2robocomp, Ice::stringToIdentity("webots2robocomp"));
			adapterWebots2Robocomp->activate();
			std::cout << "[" << PROGRAM_NAME << "]: Webots2Robocomp adapter created in port " << tmp << std::endl;
		}
		catch (const IceStorm::TopicExists&){
			std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Webots2Robocomp\n";
		}


		// Server adapter creation and publication
		std::shared_ptr<IceStorm::TopicPrx> joystickadapter_topic;
		Ice::ObjectPrxPtr joystickadapter;
		try
		{

		    tmp = configLoader.get<std::string>("JoystickAdapterTopic.Endpoints");
			Ice::ObjectAdapterPtr JoystickAdapter_adapter = communicator()->createObjectAdapterWithEndpoints("joystickadapter", tmp);
			RoboCompJoystickAdapter::JoystickAdapterPtr joystickadapterI_ =  std::make_shared <JoystickAdapterI>(worker);
			auto joystickadapter = JoystickAdapter_adapter->addWithUUID(joystickadapterI_)->ice_oneway();
			if(!joystickadapter_topic)
			{
				try {
					joystickadapter_topic = topicManager->create("JoystickAdapter");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						std::cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						joystickadapter_topic = topicManager->retrieve("JoystickAdapter");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						std::cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				catch(const IceUtil::NullHandleException&)
				{
					std::cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
					"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
					return EXIT_FAILURE;
				}
				IceStorm::QoS qos;
				joystickadapter_topic->subscribeAndGetPublisher(qos, joystickadapter);
			}
			JoystickAdapter_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			std::cout << "[" << PROGRAM_NAME << "]: Error creating JoystickAdapter topic.\n";
			//Error. Topic does not exist
		}


		// Server adapter creation and publication
		std::cout << SERVER_FULL_NAME " started" << std::endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: joystickadapter " <<std::endl;
			joystickadapter_topic->unsubscribe( joystickadapter );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: joystickadapter " << ex.what()<<std::endl;
		}


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		std::cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << std::endl;
		std::cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	delete worker;
	return status;
}

int main(int argc, char* argv[])
{
	std::string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				std::cout << "Startup check = True"<< std::endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	::Webots2Robocomp app(configFile, prefix, startup_check_flag);

	return app.main(argc, argv);
}
