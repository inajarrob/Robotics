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
		case State::raiseArm:
			raiseArm();
		break;
	}
}

void SpecificWorker::idle()
{
	qDebug() << __FUNCTION__;
	// si estan vacios buscamos una caja
	if(visitedTags.datos.empty() and handTags.datos.empty()){
		actual_state = State::turn;
		iter += 1;
		qDebug() << "Iter: " << iter;
	}
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
		// salto a centrar mano
	auto [exists, tp] = handTags.readSelected(current_id);
	if(exists)
	{
		try
		{
			gotopoint_proxy->stop();
			actual_state = State::focus;
			
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
		}	
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
	qDebug() << "Bool: " << exists;
	auto [id, x, z, ry, camera] = tp;
	auto r = std::get<1>(tp);
	//auto r = std::get<int>(tp);
	

 	if(fabs(x) < 10 and fabs(ry) < 10){
		actual_state = State::moveArm;
	}	
	else
	{
		qDebug() << "Enfocando";
		if(fabs(x) >= 10){
			if(x < 0){
				try {
					qDebug() << "Enfocando -x: " << x;
					Pose6D pose = {increment,0,0,0,0,0};
					simplearm_proxy->moveTo(pose);
				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
			}
			else{
				try {
					qDebug() << "Enfocando x: " << x;
					Pose6D pose = {-increment,0,0,0,0,0};
					simplearm_proxy->moveTo(pose);
				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
			}
		}
		if(fabs(ry) >= 10){
			if(ry < 0){
				try{
					qDebug() << "Enfocando -ry: " << ry;
					Pose6D pose = {0,0,-increment,0,0,0};
					simplearm_proxy->moveTo(pose);
				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
			}
			else{
				try {
					qDebug() << "Enfocando ry: " << ry;
					Pose6D pose = {0,0,increment,0,0,0};
					simplearm_proxy->moveTo(pose);
				}
				catch(const std::exception& e)
				{
					std::cerr << e.what() << '\n';
				}
			}
		}
	} 
}

void SpecificWorker::moveArm()
{
	auto [exists, tp] = handTags.readSelected(current_id);
	auto [id, x, z, ry, camera] = tp;
	qDebug() << "moveArm: " << z;

	if (fabs(z) > 175)
	{
		Pose6D pose = {0,increment,0,0,0,0};
		qDebug() << "X: " << x << "RY: " << ry;
		simplearm_proxy->moveTo(pose);
		if(fabs(x) > 10 or fabs(ry) > 10){
			actual_state = State::focus;
		}
	}
	else 
	{
		if(fabs(z) <= 175){
			qDebug() << "ALCANZADO X: " << x << "RY: " << ry;
			simplearm_proxy->stop();
			//sleep(2000);
			qDebug() << " EH LETS GO";
			actual_state = State::raiseArm;
		}
	}
}


void SpecificWorker::raiseArm(){
	qDebug() << "tamos en raise bro";
	visitedIds.push_back(std::get<0>(handTags.datos[0]));
	auto [exists, tp] = handTags.readSelected(current_id);
	auto [id, x, z, ry, camera] = tp;

	if(z > 200){
		actual_state = State::idle;
		simplearm_proxy->stop();
		handTags.datos.clear();
		visitedTags.datos.clear();
	} else{
		qDebug() << "Z en raiseArm: " << z;
		Pose6D pose = {0,-increment,0,0,0,0};
		simplearm_proxy->moveTo(pose);
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
			//qDebug() << __FUNCTION__ << "dentro";
			tps.push_back(std::make_tuple(v.id, v.tx, v.tz, v.ry, v.cameraId));
			aux = v.cameraId;
			if(aux == "rgbd")
			{
				//qDebug() << __FUNCTION__ << "write";
				visitedTags.write(tps);
			}
			
			else
			{
				if(aux == "rgbdHand"){
					//qDebug() << __FUNCTION__ << "hand";
					handTags.write(tps);
					//gotopoint_proxy->stop();
				}
			//z = std::get<2>(handTags.datos[0]);
			}
		}
	}
}


void SpecificWorker::AprilTags_newAprilTagAndPose(tagsList tags, RoboCompGenericBase::TBaseState bState, RoboCompJointMotor::MotorStateMap hState)
{
//subscribesToCODE

}



