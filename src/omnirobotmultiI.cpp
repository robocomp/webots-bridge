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
#include "omnirobotmultiI.h"

OmniRobotMultiI::OmniRobotMultiI(GenericWorker *_worker)
{
	worker = _worker;
}


OmniRobotMultiI::~OmniRobotMultiI()
{
}


void OmniRobotMultiI::getBasePose(int robotId, int &x, int &z, float &alpha, const Ice::Current&)
{
	worker->OmniRobotMulti_getBasePose(robotId, x, z, alpha);
}

void OmniRobotMultiI::setSpeedBase(int robotId, float advx, float advz, float rot, const Ice::Current&)
{
	worker->OmniRobotMulti_setSpeedBase(robotId, advx, advz, rot);
}

void OmniRobotMultiI::stopBase(int robotId, const Ice::Current&)
{
	worker->OmniRobotMulti_stopBase(robotId);
}

