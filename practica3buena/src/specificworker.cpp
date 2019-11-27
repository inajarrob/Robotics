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
	s = -((log(a))/(b*b));
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

void SpecificWorker::idle()
{
	if(c.isActive())
	{
		SpecificWorker::actual_state = SpecificWorker::State::goToAndWalk;
	}
}

// Orientarse
void SpecificWorker::goToAndWalk(){
	// hasta que no este orientado al pick sigue girando	
	r = innerModel->transform("base", QVec::vec3(c.pick.x, 0, c.pick.z), "world");
		
	// hacer arcotangente para saber rotacion
	rot = atan2(r.x(),r.z());
	/*double speed;
	float hipotenusa = sqrt(c.a*c.a+c.b*c.b+c.n*c.n);
	if(hipotenusa >800){
		hipotenusa = 800;
	}
	speed = hipotenusa*(1/100);*/

	 

	/*double gauss = exp((-s*(rot*rot)));
	double distTarget;
	if(d > 1000){
		distTarget = 1;
	} 
	else{
		distTarget = d*(1/1000);
	} 
	//double distTarget = (1/(1+exp(-rot)));
	forwardSpeed = 800*gauss*distTarget;*/

	// Sort from min to max distance to objects or wall
	auto v = ldata;
    std::sort(v.begin(), v.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

	if(v.front().dist < threshold)
		actual_state = State::turn;
	
	if(fabs(rot) > 1){
		// Robot orientado
		if(fabs(rot) < 0.05) {
			differentialrobot_proxy->setSpeedBase(0,0);
			return;
		}
		differentialrobot_proxy->setSpeedBase(0,rot);
	} else{
		if(checkInTarget()){
			differentialrobot_proxy->setSpeedBase(0,0);
			c.active.store(false);
			actual_state = State::idle;
		} else{
			if(d > 800)
				differentialrobot_proxy->setSpeedBase(800, rot);
			else
				differentialrobot_proxy->setSpeedBase(d, rot);
			}
	}
}

bool SpecificWorker::checkInTarget(){
	auto x = abs(c.pick.x - bState.x);
	auto z = abs(c.pick.z - bState.z);
	d = sqrt((x*x) + (z*z));
	cout << "DISTANCIA: " << d << endl;
	return (d<=150);
}

void SpecificWorker::turn(){
	auto v = ldata;
	std::sort(v.begin(), v.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

	if((fabs(v[0].angle) >= 1.45) && (fabs(v[0].angle) <= 1.60)){
		actual_state = State::skirt;
	} else {
		differentialrobot_proxy->setSpeedBase(0, 0.3);
	}
	
}

void SpecificWorker::skirt(){
	// MEJORAR: tenemos que bordear el objeto que tenemos a la izda o la dcha
	auto v = ldata;
	std::sort(v.begin()+50, v.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
	cout << v[50].dist << endl;
	if(v[50].dist < 550){
		differentialrobot_proxy->setSpeedBase(200, 0);
	} else {
		if(targetVisible()){
			actual_state = State::goToAndWalk;
			return;
		}
		if(inLine()){
			actual_state = State::goToAndWalk;
			return;
		}
		//if(ldata.)
		differentialrobot_proxy->setSpeedBase(0, -0.3);
	}
	
}

bool SpecificWorker::targetVisible(){
	bool visible = false;
	QPolygonF polygon;
	auto x = innerModel->getNode<InnerModelLaser>(std::string("laser"));
	
	for(const auto &l: ldata)
	{  
		auto r = x->laserTo(std::string("world"), l.dist, l.angle);
		polygon << QPointF(r.x(), r.z());
	}
	visible = polygon.containsPoint(QPointF(c.pick.x, c.pick.z), Qt::OddEvenFill);
	cout << "  ES VISIBLE!!    " << visible << endl;
	if(visible){
		float dist = (QVec::vec3(bState.x,0,bState.z)-r).norm2();
		auto ml0 = innerModel->transform("world", QVec::vec3(-200, 0, 0), "base");
		auto ml = innerModel->transform("world", QVec::vec3(-200, 0, dist), "base");
		auto mr0 = innerModel->transform("world", QVec::vec3(200, 0, 0), "base");
		auto mr = innerModel->transform("world", QVec::vec3(200, 0, dist), "base");
		//QLineF right = QLineF(200, 0, 200, dist);
		QLineF left = QLineF(ml0.toQPointF(), ml.toQPointF());
		QLineF right = QLineF(mr0.toQPointF(), mr.toQPointF());
		QLineF middle = QLineF(QPointF(bState.x, bState.z), QPointF(r.x(),r.z()));
		float distR = dist/400; //numero de veces que vamos a recorrer la linea
		float delta = 1.f/distR; // intervalos que vamos a recorrer
		for(float i=0; i<1;i+=delta){
			if(!polygon.containsPoint(left.pointAt(i),Qt::OddEvenFill) or
			   !polygon.containsPoint(middle.pointAt(i),Qt::OddEvenFill) or
			   !polygon.containsPoint(right.pointAt(i),Qt::OddEvenFill)){
				    visible = false;
					break;
			   }
		}
	}
	cout << "      " << visible << endl;
	return visible;
}

// pasas pos robot y sustituyes en la ecuacion y devuelve si estas en la recta 
bool SpecificWorker::inLine(){
	cout << "++++++++++++++++ " << fabs(c.a*bState.x + c.b*bState.z + c.n) << endl;
	float res = fabs(c.a*bState.x + c.b*bState.z + c.n);
	float hipotenusa = sqrt(c.a*c.a+c.b*c.b+c.n*c.n);
	return (res/hipotenusa < 400);
}

void SpecificWorker::RCISMousePicker_setPick(Pick myPick)
{
//subscribesToCODE
	auto r2 = innerModel->transform("base", QVec::vec3(c.pick.x, 0, c.pick.z), "world");
	c.setCoords(myPick, bState, r2);
	actual_state = State::idle;
}


// HAY QUE ARREGLAR QUE NO ENTRE EN LOS OBJETOS, EL GIRO QUE HEMOS HECHO AYER, LA VELOCIDAD (GAUSS). 