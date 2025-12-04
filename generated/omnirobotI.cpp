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
#include "omnirobotI.h"

OmniRobotI::OmniRobotI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	correctOdometerHandlers = {
		[this](auto a, auto b, auto c) { return worker->OmniRobot_correctOdometer(a, b, c); }
	};

	getBasePoseHandlers = {
		[this](auto a, auto b, auto c) { return worker->OmniRobot_getBasePose(a, b, c); }
	};

	getBaseStateHandlers = {
		[this](auto a) { return worker->OmniRobot_getBaseState(a); }
	};

	resetOdometerHandlers = {
		[this]() { return worker->OmniRobot_resetOdometer(); }
	};

	setOdometerHandlers = {
		[this](auto a) { return worker->OmniRobot_setOdometer(a); }
	};

	setOdometerPoseHandlers = {
		[this](auto a, auto b, auto c) { return worker->OmniRobot_setOdometerPose(a, b, c); }
	};

	setSpeedBaseHandlers = {
		[this](auto a, auto b, auto c) { return worker->OmniRobot_setSpeedBase(a, b, c); }
	};

	stopBaseHandlers = {
		[this]() { return worker->OmniRobot_stopBase(); }
	};

}


OmniRobotI::~OmniRobotI()
{
}


void OmniRobotI::correctOdometer(int x, int z, float alpha, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < correctOdometerHandlers.size())
		 correctOdometerHandlers[id](x, z, alpha);
	else
		throw std::out_of_range("Invalid correctOdometer id: " + std::to_string(id));

}

void OmniRobotI::getBasePose(int &x, int &z, float &alpha, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getBasePoseHandlers.size())
		 getBasePoseHandlers[id](x, z, alpha);
	else
		throw std::out_of_range("Invalid getBasePose id: " + std::to_string(id));

}

void OmniRobotI::getBaseState(RoboCompGenericBase::TBaseState &state, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getBaseStateHandlers.size())
		 getBaseStateHandlers[id](state);
	else
		throw std::out_of_range("Invalid getBaseState id: " + std::to_string(id));

}

void OmniRobotI::resetOdometer(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < resetOdometerHandlers.size())
		 resetOdometerHandlers[id]();
	else
		throw std::out_of_range("Invalid resetOdometer id: " + std::to_string(id));

}

void OmniRobotI::setOdometer(RoboCompGenericBase::TBaseState state, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setOdometerHandlers.size())
		 setOdometerHandlers[id](state);
	else
		throw std::out_of_range("Invalid setOdometer id: " + std::to_string(id));

}

void OmniRobotI::setOdometerPose(int x, int z, float alpha, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setOdometerPoseHandlers.size())
		 setOdometerPoseHandlers[id](x, z, alpha);
	else
		throw std::out_of_range("Invalid setOdometerPose id: " + std::to_string(id));

}

void OmniRobotI::setSpeedBase(float advx, float advz, float rot, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setSpeedBaseHandlers.size())
		 setSpeedBaseHandlers[id](advx, advz, rot);
	else
		throw std::out_of_range("Invalid setSpeedBase id: " + std::to_string(id));

}

void OmniRobotI::stopBase(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < stopBaseHandlers.size())
		 stopBaseHandlers[id]();
	else
		throw std::out_of_range("Invalid stopBase id: " + std::to_string(id));

}

