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
#ifndef OMNIROBOTMULTI_H
#define OMNIROBOTMULTI_H

// Ice includes
#include <Ice/Ice.h>
#include <OmniRobotMulti.h>

#include <config.h>
#include "genericworker.h"


class OmniRobotMultiI : public virtual RoboCompOmniRobot::OmniRobotMulti
{
public:
	OmniRobotMultiI(GenericWorker *_worker);
	~OmniRobotMultiI();

	void getBasePose(int robotId, int &x, int &z, float &alpha, const Ice::Current&);
	void setSpeedBase(int robotId, float advx, float advz, float rot, const Ice::Current&);
	void stopBase(int robotId, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
