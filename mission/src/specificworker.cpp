/*
 *    Copyright (C)2019 by YOUR NAME HERE
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

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx) : GenericWorker(tprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//       THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = new InnerModel(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }



	


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::compute()
{
//computeCODE
//QMutexLocker locker(mutex);
//	try
//	{
//		camera_proxy->getYImage(0,img, cState, bState);
//		memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
//		searchTags(image_gray);
//	}
//	catch(const Ice::Exception &e)
//	{
//		std::cout << "Error reading from Camera" << e << std::endl;
//	}
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	ldata = laser_proxy->getLaserData();

	switch(actual_state){
		case State::idle:
			idle();
		break;
		case State::goToAndWalk:
			GotoPoint_go();
		break;
		case State::turn:
			GotoPoint_turn(0.3);
		break;
		case State::skirt:
			skirt();
		break;
	}
}




bool SpecificWorker::GotoPoint_atTarget()
{
	//implementCODE
	auto x = abs(c.pick.x - bState.x);
	auto z = abs(c.pick.z - bState.z);
	d = sqrt((x*x) + (z*z));
	return (d<=150);
}

void SpecificWorker::GotoPoint_go(string nodo, float x, float y, float alpha)
{
	//implementCODE
	// hasta que no este orientado al pick sigue girando	
	r = innerModel->transform("base", QVec::vec3(c.pick.x, 0, c.pick.z), "world");
		
	// hacer arcotangente para saber rotacion
	rot = atan2(r.x(),r.z());

	double distTarget = (1/(1+exp(-r.norm2())))-0.5; // sinusoide
	double gauss = exp((-s*(rot*rot)));

	forwardSpeed = 600*gauss*distTarget;

	// Sort from min to max distance to objects or wall
	auto v = ldata;
    std::sort(v.begin(), v.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

	if(v.front().dist < threshold){
		actual_state = State::turn;
		cout << "Saltamos a TURN" << endl;
		//return;
	}
		
	if(fabs(rot) > 1){
		// Robot orientado
		if(fabs(rot) < 0.05) {
			differentialrobot_proxy->setSpeedBase(0,0);
			return;
		}
		cout << "Orientamos a: " << rot << endl;
		differentialrobot_proxy->setSpeedBase(0,rot);
	} else{
		if(checkInTarget()){
			differentialrobot_proxy->setSpeedBase(0,0);
			c.active.store(false);
			actual_state = State::idle;
		} else{
			cout << "Andando" << endl;
			if(forwardSpeed > 800)
				differentialrobot_proxy->setSpeedBase(800, rot);
			else
				differentialrobot_proxy->setSpeedBase(forwardSpeed, rot);
			}
	}
}

void SpecificWorker::GotoPoint_stop()
{
	//implementCODE
	differentialrobot_proxy->setSpeedBase(0, 0);
}

void SpecificWorker::GotoPoint_turn(float speed)
{
	//implementCODE
	auto v = ldata;
	std::sort(v.begin(), v.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

	if((fabs(v[0].angle) >= 1.45) && (fabs(v[0].angle) <= 1.60)){
		turning = false;
		cout << "Saltamos a SKIRT" << endl;
		actual_state = State::skirt;
	} else {
		turning = true;
		cout << "Girando a 0.3 en turn" << endl;
		differentialrobot_proxy->setSpeedBase(1, speed);
	}

}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	//subscribesToCODE
	auto r2 = innerModel->transform("base", QVec::vec3(c.pick.x, 0, c.pick.z), "world");
	c.setCoords(myPick, bState, r2);
	actual_state = State::idle;
}


