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
//	catch(std::exception e) { qFatal("Error reading config params"); }


	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	timer.start(Period);

}

void SpecificWorker::walk(RoboCompLaser::TLaserData ldata)
{
    int x, y;
    float alpha;

    differentialrobot_proxy->setSpeedBase(500, 0);
    /* std::cout << ".................CORRIENDO.................." << std::endl;
    std::cout << "DISTANCIA: " << ldata.front().dist << std::endl;
    differentialrobot_proxy->getBasePose(x, y, alpha);
    std::cout << "Posicion, x: " << x << "y: " << y << std::endl;
    std::cout << "ANGULO: " << ldata.front().angle << std::endl;
    */

    if( ldata.front().dist < threshold)
        setState(State::findObj);
}

	void SpecificWorker::setState(State a_state){
        actual_state = a_state;
    }
    
	enum State SpecificWorker::getState(){
        return actual_state;
    }

void SpecificWorker::compute( )
{

    try
    {
    	// read laser data 
        RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData(); 


        //sort laser data from small to large distances using a lambda function.
        // ORDENA DE MENOR A MAYOR DISTANCIA A OBJETOS/PARED
        //std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

        switch(actual_state)
        {
            case State::idle:
                break;
            case State::walk:
                SpecificWorker::walk(ldata, enum actual_state);
                break;
            case State::turn:
                break;
            case State::findObj:
                    findObstacles(ldata, enum actual_state);
                break;
        }
    }
	/* if( ldata.front().dist < threshold)
    {
                std::cout << ".................CAMBIO DIRECCION.................." << std::endl;
                //differentialrobot_proxy->stopBase();
                std::cout << "DISTANCIA: " << ldata.front().dist << std::endl;
                differentialrobot_proxy->getBasePose(x, y, alpha);
                std::cout << "Posicion, x: " << x << "y: " << y << std::endl;
                std::cout << "ANGULO: " << ldata.front().angle << std::endl;
                differentialrobot_proxy->setSpeedBase(5, 0.6);
                usleep(rand()%(1500000-100000 + 1) + 100000);  // random wait between 1.5s and 0.1sec
    }
	else
	{
        differentialrobot_proxy->setSpeedBase(500, 0);
        std::cout << ".................CORRIENDO.................." << std::endl;
        std::cout << "DISTANCIA: " << ldata.front().dist << std::endl;
        differentialrobot_proxy->getBasePose(x, y, alpha);
        std::cout << "Posicion, x: " << x << "y: " << y << std::endl;
        std::cout << "ANGULO: " << ldata.front().angle << std::endl;
  	}
    }*/
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    } 
}




