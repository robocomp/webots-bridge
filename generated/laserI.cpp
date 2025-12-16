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
#include "laserI.h"

LaserI::LaserI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getLaserAndBStateDataHandlers = {
		[this](auto a) { return worker->Laser_getLaserAndBStateData(a); }
	};

	getLaserConfDataHandlers = {
		[this]() { return worker->Laser_getLaserConfData(); }
	};

	getLaserDataHandlers = {
		[this]() { return worker->Laser_getLaserData(); }
	};

}


LaserI::~LaserI()
{
}


RoboCompLaser::TLaserData LaserI::getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLaserAndBStateDataHandlers.size())
		return  getLaserAndBStateDataHandlers[id](bState);
	else
		throw std::out_of_range("Invalid getLaserAndBStateData id: " + std::to_string(id));

}

RoboCompLaser::LaserConfData LaserI::getLaserConfData(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLaserConfDataHandlers.size())
		return  getLaserConfDataHandlers[id]();
	else
		throw std::out_of_range("Invalid getLaserConfData id: " + std::to_string(id));

}

RoboCompLaser::TLaserData LaserI::getLaserData(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getLaserDataHandlers.size())
		return  getLaserDataHandlers[id]();
	else
		throw std::out_of_range("Invalid getLaserData id: " + std::to_string(id));

}

