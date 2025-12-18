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
#include "webots2robocompI.h"

Webots2RobocompI::Webots2RobocompI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	resetWebotsHandlers = {
		[this]() { return worker->Webots2Robocomp_resetWebots(); }
	};

	setDoorAngleHandlers = {
		[this](auto a) { return worker->Webots2Robocomp_setDoorAngle(a); }
	};

	setPathToHumanHandlers = {
		[this](auto a, auto b) { return worker->Webots2Robocomp_setPathToHuman(a, b); }
	};

}


Webots2RobocompI::~Webots2RobocompI()
{
}


void Webots2RobocompI::resetWebots(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < resetWebotsHandlers.size())
		 resetWebotsHandlers[id]();
	else
		throw std::out_of_range("Invalid resetWebots id: " + std::to_string(id));

}

void Webots2RobocompI::setDoorAngle(float angle, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setDoorAngleHandlers.size())
		 setDoorAngleHandlers[id](angle);
	else
		throw std::out_of_range("Invalid setDoorAngle id: " + std::to_string(id));

}

void Webots2RobocompI::setPathToHuman(int humanId, RoboCompGridder::TPath path, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setPathToHumanHandlers.size())
		 setPathToHumanHandlers[id](humanId, path);
	else
		throw std::out_of_range("Invalid setPathToHuman id: " + std::to_string(id));

}

