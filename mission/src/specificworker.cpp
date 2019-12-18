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
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		std::string innermodel_path = par.value;
		innerModel = std::make_shared<InnerModel>(par.value);
	
	}
	catch(const std::exception &e) { qFatal("Error reading config params"); }
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
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
		switch(actual_state){
			case State::idle:
				idle();
			break;
			case State::goToAndWalk:
				goToAndWalk();
			break;
			case State::turn:
				turn();
			break;
			case State::skirt:
				skirt();
			break;
		}
	
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Robot" << e << std::endl;
	}
}

bool SpecificWorker::GotoPoint_atTarget()
{
	auto x = abs(c.x - bState.x);
	auto z = abs(c.z - bState.z);
	auto d = sqrt((x*x) + (z*z));
	return (d<=150);
}

void SpecificWorker::GotoPoint_go(string nodo, float x, float y, float alpha)
{
	qDebug() << "al llegar" << x << y;
	auto r = innerModel->transform("world",QVec::vec3(x,0,y), "rgbd");
	qDebug() << "al transformar" << r.x() << r.z();
	exit(-1);
	c.setCoords(r.x(), r.z(), alpha);
	//differentialrobot_proxy->setSpeedBase(400, 0);
	actual_state = State::goToAndWalk;
}

void SpecificWorker::GotoPoint_stop()
{
	differentialrobot_proxy->setSpeedBase(0, 0);
}

void SpecificWorker::GotoPoint_turn(float speed)
{
	if(speed > 1){
		speed = 1;
	} 
	if(speed < -1){
		speed = -1;
	}
	differentialrobot_proxy->setSpeedBase(0, speed);
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
	//subscribesToCODE
}

void SpecificWorker::idle()
{
	if(c.isActive())
	{
		SpecificWorker::actual_state = SpecificWorker::State::goToAndWalk;
	}
}

// Orientarse
void SpecificWorker::goToAndWalk(){
	r = innerModel->transform("rgbd", QVec::vec3(c.x, 0, c.z), "world");
	rot = atan2(r.x(),r.z());
	forwardSpeed = 400;
		
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

bool SpecificWorker::checkInTarget(){
	auto x = abs(c.x - bState.x);
	auto z = abs(c.z - bState.z);
	d = sqrt((x*x) + (z*z));
	return (d<=150);
}

void SpecificWorker::turn(){
	differentialrobot_proxy->setSpeedBase(1, 0.3);	
}

// pasas pos robot y sustituyes en la ecuacion y devuelve si estas en la recta 
bool SpecificWorker::inLine(){
	/*float res = fabs(c.a*bState.x + c.b*bState.z + c.n);
	float hipotenusa = sqrt(c.a*c.a+c.b*c.b+c.n*c.n);
	return (res/hipotenusa < 400);*/
	return true;
}

void SpecificWorker::skirt(){
	
}


bool SpecificWorker::targetVisible(){
	/*bool visible = false;
	QPolygonF polygon;
	auto x = innerModel->getNode<InnerModelLaser>(QString("laser"));
	
	for(const auto &l: ldata)
	{  
		auto r = x->laserTo(QString("world"), l.dist, l.angle);
		polygon << QPointF(c.x(), c.z());
	}
	visible = polygon.containsPoint(QPointF(c.pick.x, c.pick.z), Qt::OddEvenFill);
	//cout << "  ES VISIBLE!!    " << visible << endl;
	if(visible){
		float dist = (QVec::vec3(bState.x,0,bState.z)-QVec::vec3(c.pick.x,0, c.pick.z)).norm2();
		auto tr = innerModel->transform("base", QVec::vec3(c.pick.x, 0, c.pick.z), "world");
		auto ml0 = innerModel->transform("world", QVec::vec3(-200, 0, 0), "base");
		auto ml = innerModel->transform("world", QVec::vec3(tr.x()-200, 0, tr.z()), "base");
		auto mr0 = innerModel->transform("world", QVec::vec3(200, 0, 0), "base");
		auto mr = innerModel->transform("world", QVec::vec3(tr.x()+200, 0, tr.z()), "base");

		QLineF left = QLineF(QPointF(ml0.x(), ml0.z()), QPointF(ml.x(), ml.z()));
		QLineF right = QLineF(QPointF(mr0.x(), mr0.z()), QPointF(mr.x(), mr.z()));
		QLineF middle = QLineF(QPointF(bState.x, bState.z), QPointF(c.pick.x,c.pick.z));
		float distR = dist/400; //numero de veces que vamos a recorrer la linea
		float delta = 1.f/distR; // intervalos que vamos a recorrer
		for(float i=delta; i<1;i+=delta)
		{

			//cout << "i: " << i << " Izda: " << polygon.containsPoint(left.pointAt(i),Qt::OddEvenFill) << " Centro: " << polygon.containsPoint(middle.pointAt(i),Qt::OddEvenFill) << " Dcha: " << polygon.containsPoint(right.pointAt(i),Qt::OddEvenFill) << endl;
			if(!polygon.containsPoint(left.pointAt(i),Qt::OddEvenFill) or
			   !polygon.containsPoint(middle.pointAt(i),Qt::OddEvenFill) or
			   !polygon.containsPoint(right.pointAt(i),Qt::OddEvenFill))
			   {
				    visible = false;
					//cout << "NO VISIBLE" << endl;
					break;
			   }
		}
		
	}
	//cout << "      " << visible << endl;
	return visible;*/
}





