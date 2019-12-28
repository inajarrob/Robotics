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
	actual_state = State::idle;
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
	switch(actual_state){
		case State::idle:
			idle();
		break;
		case State::turn:
			turn();
		break;
		case State::check_target:
			check_target();
		break;
		case State::focus:
			focus();
		break;
		case State::moveArm:
			moveArm();
		break;
	}
}

void SpecificWorker::idle()
{
	qDebug() << __FUNCTION__;
	actual_state = State::turn;

}

void SpecificWorker::turn()
{
	qDebug() << __FUNCTION__;
	try
	{
		if(visitedTags.read().empty() == false)
		{ 
			gotopoint_proxy->stop();
			auto [id,x,z,ry,idCamera] = visitedTags.read()[0];
			qDebug() << "to gotopoint_proxy" << x << z;
			gotopoint_proxy->go("",x,z,ry);
			current_id = id;
			actual_state = State::check_target;
		}
		else {
			gotopoint_proxy->turn(0.3);
		}
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
}

void SpecificWorker::check_target()
{
	qDebug() << __FUNCTION__;
	// si visitedTagHand recibido por cameraHand (id==rgbdHand)
		// paro
		// salto a centrrar mano
	auto [exists, tp] = handTags.readSelected(current_id);
	if(exists)
	{
		gotopoint_proxy->stop();
		actual_state = State::focus;
	}
	if(gotopoint_proxy->atTarget())
	{
		gotopoint_proxy->stop();
		actual_state = State::idle;
	} 
}

/// Centrar mano
void SpecificWorker::focus()
{
	//si fabs(t.x) < 10 and fabs(x)<10
	//	cambio a bajarMano

	auto [exists, tp] = handTags.readSelected(current_id);
	auto [id, x, z, ry, camera] = tp;
	auto r = std::get<1>(tp);
	//auto r = std::get<int>(tp);
	

 	if(fabs(x) < 10 and fabs(z) < 10){
		actual_state = State::moveArm;
	}	
	else
	{
		if(fabs(x) >= 10){
			if(x < 0){
				Pose6D pose = {increment,0,0,0,0,0};
				simplearm_proxy->moveTo(pose);
			}
			else{
				Pose6D pose = {-increment,0,0,0,0,0};
				simplearm_proxy->moveTo(pose);
			}
		}
		if(fabs(z) >= 10){
			if(z < 0){
				Pose6D pose = {0,increment,0,0,0,0};
				simplearm_proxy->moveTo(pose);
			}
			else{
				Pose6D pose = {0,-increment,0,0,0,0};
				simplearm_proxy->moveTo(pose);
			}
		}
	} 
}

void SpecificWorker::moveArm()
{
	/* if (fabs(handTags.read()[0]<3>) > 50)
	{
		void moveTo (Pose6D pose);
		//simplearm_proxy->moveTo(Pose6D(0,0,-increment, 0,0,0));
		//simplearm_proxy->moveTo(QVec::vec6(0,0,-increment,0,0,0));
	}
	else 
	{
		if(fabs(handTags.read()[0]<3>) <= 50){
			sleep(2000);
			actual_state = State::idle;
			visitedIds.push_back(handTags.read()[0]<0>);
		}
	} */
	if (fabs(std::get<3>(handTags.datos[0])) > 50)
	{
	
		//auto tuple = std::make_tuple(0.0,0.0,-increment,0.0,0.0,0);
		Pose6D pose = {0,0,-increment,0,0,0};
		simplearm_proxy->moveTo(pose);
	}
	else 
	{
		if(fabs(std::get<3>(handTags.datos[0])) <= 50){
			sleep(2000);
			actual_state = State::idle;
			visitedIds.push_back(std::get<0>(handTags.datos[0]));
		}
	}
}




//////////////////////////////////////////////////////////

// la marca va en referencia al mundo, apriltag en el robot
void SpecificWorker::AprilTags_newAprilTag(tagsList tags)
{
	std::vector<Tp> tps;
	std::string aux = "";
	for(const auto &v : tags)
	{
		// de 0 a 10 cajas pared
		// de 10 a 20 cajas suelo
		//qDebug() << v.cameraId;
		if(v.id > 10 
			and 
			std::find(std::begin(visitedIds), std::end(visitedIds), v.id) == std::end(visitedIds))
		{
			tps.push_back(std::make_tuple(v.id, v.tx, v.tz, v.ry, v.cameraId));
			aux = v.cameraId;
		}
	}
	if(aux == "rgbd")
		visitedTags.write(tps);
	else
		handTags.write(tps);
}

void SpecificWorker::AprilTags_newAprilTagAndPose(tagsList tags, RoboCompGenericBase::TBaseState bState, RoboCompJointMotor::MotorStateMap hState)
{
//subscribesToCODE

}



