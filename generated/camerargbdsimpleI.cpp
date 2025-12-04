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
#include "camerargbdsimpleI.h"

CameraRGBDSimpleI::CameraRGBDSimpleI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	getAllHandlers = {
		[this](auto a) { return worker->CameraRGBDSimple_getAll(a); }
	};

	getDepthHandlers = {
		[this](auto a) { return worker->CameraRGBDSimple_getDepth(a); }
	};

	getImageHandlers = {
		[this](auto a) { return worker->CameraRGBDSimple_getImage(a); }
	};

	getPointsHandlers = {
		[this](auto a) { return worker->CameraRGBDSimple_getPoints(a); }
	};

}


CameraRGBDSimpleI::~CameraRGBDSimpleI()
{
}


RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimpleI::getAll(std::string camera, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getAllHandlers.size())
		return  getAllHandlers[id](camera);
	else
		throw std::out_of_range("Invalid getAll id: " + std::to_string(id));

}

RoboCompCameraRGBDSimple::TDepth CameraRGBDSimpleI::getDepth(std::string camera, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getDepthHandlers.size())
		return  getDepthHandlers[id](camera);
	else
		throw std::out_of_range("Invalid getDepth id: " + std::to_string(id));

}

RoboCompCameraRGBDSimple::TImage CameraRGBDSimpleI::getImage(std::string camera, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getImageHandlers.size())
		return  getImageHandlers[id](camera);
	else
		throw std::out_of_range("Invalid getImage id: " + std::to_string(id));

}

RoboCompCameraRGBDSimple::TPoints CameraRGBDSimpleI::getPoints(std::string camera, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getPointsHandlers.size())
		return  getPointsHandlers[id](camera);
	else
		throw std::out_of_range("Invalid getPoints id: " + std::to_string(id));

}

