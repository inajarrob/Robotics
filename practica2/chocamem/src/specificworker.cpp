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

 static int walkc = 0;
 static bool t = false;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	setState(State::idle);
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
	
	RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
	innerModel = std::make_shared<InnerModel>(par.value);
	xmin = std::stoi(params.at("xmin").value);
	xmax = std::stoi(params.at("xmax").value);
	ymin = std::stoi(params.at("ymin").value);
	ymax = std::stoi(params.at("ymax").value);
	tilesize = std::stoi(params.at("tilesize").value);

	// Scene
 	scene.setSceneRect(xmin, ymin, fabs(xmin)+fabs(xmax), fabs(ymin)+fabs(ymax));
 	view.setScene(&scene);
 	view.scale(1, -1);
 	view.setParent(scrollArea);
 	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );
	grid.initialize( TDim{ tilesize, xmin, xmax, ymin, ymax}, TCell{true, false, nullptr} );

	qDebug() << "Grid initialize ok";

	for(auto &[key, value] : grid)
 	{
	 	value.rect = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));			
		value.rect->setPos(key.x,key.z);
	}

 	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
 	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
 	noserobot->setBrush(Qt::magenta);

	view.show();
	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	this->Period = period;
	timer.start(period);
	qDebug() << "End initialize";

}

void SpecificWorker::compute()
{
	// read laser data 
	//ldata = laser_proxy->getLaserData(); 
    readRobotState(ldata);

    switch(SpecificWorker::actual_state)
    {
        case State::idle:
            idle();
        break;
        case State::walk:
            walk(ldata);
        break;
        case State::turn:
            turn(ldata);
        break;
        case State::randTurn:
            randTurn(ldata);
        break;
    }
}

void SpecificWorker::readRobotState(RoboCompLaser::TLaserData &ldata)
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);

		//update occupied cells
		updateOccupiedCells(bState, ldata);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading from Laser" << e << std::endl;
	}
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
	 		view.setFixedSize(scrollArea->width(), scrollArea->height());
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	InnerModelLaser *n = innerModel->getNode<InnerModelLaser>(QString("laser"));
	for(auto l: ldata)
	{
		auto r = n->laserTo(QString("world"), l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
		{
			cell.free = false;
			cell.rect->setBrush(Qt::darkRed);
		}
	}

	auto [valid, cell] = grid.getCell(bState.x, bState.z);
	if(!(cell.visited)){
		cell.visited = true;
		cell.rect->setBrush(Qt::black);
	}

}

void SpecificWorker::setState(SpecificWorker::State a_state){
    SpecificWorker::actual_state = a_state;
} 

void SpecificWorker::idle(){
    setState(SpecificWorker::State::walk);
}

void SpecificWorker::walk(RoboCompLaser::TLaserData ldata)
{
    // ORDENA DE MENOR A MAYOR DISTANCIA A OBJETOS/PARED
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    //static int walkc = 0;
	/*differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	auto [valid, cell] = grid.getCell(bState.x, bState.z);*/

    if(ldata.front().dist < threshold){
        t = true;
        setState(SpecificWorker::State::turn);   
           
    }
    else{
        if(ldata.front().dist < 400){
            differentialrobot_proxy->setSpeedBase(500, 0);
            walkc++;
        }else{
        	differentialrobot_proxy->setSpeedBase(1000, 0);
            walkc++;
        }
    }
    if(walkc>40){
        walkc = 0;
        setState(SpecificWorker::State::randTurn);
    }
}

void SpecificWorker::turn(RoboCompLaser::TLaserData ldata)
{
    
    // ORDENA DE MENOR A MAYOR DISTANCIA A OBJETOS/PARED
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });
	
    if (t == true){
		if(ldata.front().dist < 100 && ldata.back().dist < 100){
			cout << "++++++++Esquina+++++++++" << endl;
			differentialrobot_proxy->setSpeedBase(0, rot);
			usleep(1000000);
		}
		if(ldata.front().angle > 0){
			differentialrobot_proxy->setSpeedBase(0, -rot);
        	usleep(rand()%(1500000-100000+1) + 100000);  
		}else{
			differentialrobot_proxy->setSpeedBase(0, rot);
        	usleep(rand()%(1500000-100000+1) + 100000);  
	}      
        t = false;
    }	
    else
        setState(SpecificWorker::State::walk);

    

} 

void SpecificWorker::randTurn(RoboCompLaser::TLaserData ldata)
{
    static int giro = 0;
    giro    = rand()%10;
    if(giro==0){
         differentialrobot_proxy->setSpeedBase(0, -rot);
    
    }
    else{
        differentialrobot_proxy->setSpeedBase(0, rot);
    }
    
    setState(SpecificWorker::State::walk);

    /*static int giro = 0;
    //giro    = rand()%2;
    //differentialrobot_proxy->stopBase();
    if(giro == 0){
        // Giro izquierda
        differentialrobot_proxy->setSpeedBase(0, rot);
        setState(SpecificWorker::State::walk);
    } else{
        // Giro izquierda
        differentialrobot_proxy->setSpeedBase(0, rot);
        setState(SpecificWorker::State::walk);
    }*/
 }



///////////////////////////////////////////////////////////////////77
////  SUBSCRIPTION
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
//subscribesToCODE

}



