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
#include "imuI.h"

IMUI::IMUI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getAccelerationHandlers = {
		[this]() { return worker->IMU_getAcceleration(); }
	};

	getAngularVelHandlers = {
		[this]() { return worker->IMU_getAngularVel(); }
	};

	getDataImuHandlers = {
		[this]() { return worker->IMU_getDataImu(); }
	};

	getMagneticFieldsHandlers = {
		[this]() { return worker->IMU_getMagneticFields(); }
	};

	getOrientationHandlers = {
		[this]() { return worker->IMU_getOrientation(); }
	};

	resetImuHandlers = {
		[this]() { return worker->IMU_resetImu(); }
	};

}


IMUI::~IMUI()
{
}


RoboCompIMU::Acceleration IMUI::getAcceleration(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getAccelerationHandlers.size())
		return  getAccelerationHandlers[id]();
	else
		throw std::out_of_range("Invalid getAcceleration id: " + std::to_string(id));

}

RoboCompIMU::Gyroscope IMUI::getAngularVel(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getAngularVelHandlers.size())
		return  getAngularVelHandlers[id]();
	else
		throw std::out_of_range("Invalid getAngularVel id: " + std::to_string(id));

}

RoboCompIMU::DataImu IMUI::getDataImu(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getDataImuHandlers.size())
		return  getDataImuHandlers[id]();
	else
		throw std::out_of_range("Invalid getDataImu id: " + std::to_string(id));

}

RoboCompIMU::Magnetic IMUI::getMagneticFields(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getMagneticFieldsHandlers.size())
		return  getMagneticFieldsHandlers[id]();
	else
		throw std::out_of_range("Invalid getMagneticFields id: " + std::to_string(id));

}

RoboCompIMU::Orientation IMUI::getOrientation(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getOrientationHandlers.size())
		return  getOrientationHandlers[id]();
	else
		throw std::out_of_range("Invalid getOrientation id: " + std::to_string(id));

}

void IMUI::resetImu(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < resetImuHandlers.size())
		 resetImuHandlers[id]();
	else
		throw std::out_of_range("Invalid resetImu id: " + std::to_string(id));

}

