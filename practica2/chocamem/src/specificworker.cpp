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
	timer.start(Period);
	qDebug() << "End initialize";

}

void SpecificWorker::compute()
{
	// read laser data 
	ldata = laser_proxy->getLaserData(); 
    readRobotState(ldata);

    switch(SpecificWorker::actual_state)
    {
        case State::idle:
            cout << "Idle" << endl;
            idle();
        break;
        case State::walk:
            cout << "Walking..." << endl;
            walk(ldata);
        break;
        case State::turn:
            cout << "Turn" << endl;
            turn(ldata);
        break;
        case State::findObj:
			cout << "Find object" << endl;
            findObstacle(ldata);
        break;
    }
}

void SpecificWorker::readRobotState(RoboCompLaser::TLaserData ldata)
{
	try
	{
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		//RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
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
    QTimer timer;
    timer.start(1000);
    QMutex mutex;
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	auto [valid, cell] = grid.getCell(bState.x, bState.z);

    if(ldata.front().dist < threshold){
        setState(SpecificWorker::State::turn);
    }
    if(walkc>40/*timer.interval() == 5000*/){ //hacer mejor con un timer
        //mutex.lock();
        walkc = 0;
        cout << "DEBERIA ESTAR GIRANDO.." << endl;
        setState(SpecificWorker::State::findObj);

        //timer.setInterval(0);
        //mutex.unlock();
    }
    else
        if(ldata.front().dist < 400){
            differentialrobot_proxy->setSpeedBase(500, 0);
            walkc++;
        }else{
        	differentialrobot_proxy->setSpeedBase(1000, 0);
            walkc++;
        }

}

void SpecificWorker::turn(RoboCompLaser::TLaserData ldata)
{
    static int giro = 0;
    static int br = 0;
    static bool turning = false;

    // ORDENA DE MENOR A MAYOR DISTANCIA A OBJETOS/PARED
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; });

    // Preguntamos si hay que terminar de girar
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
	auto [valid, cell] = grid.getCell(bState.x, bState.z);
    //if(ldata.front().dist > threshold && !(cell.visited) && (cell.free))
    if(ldata.front().dist > threshold)
    {
        setState(SpecificWorker::State::walk);
        turning = false;
    } else {
        // ¿Estabamos girando ya?
		int maxpos = 0;
		for(int i=0;i<ldata.size(); i++){
			if(abs(ldata[i].angle) > abs(ldata[maxpos].angle))
				maxpos = i;
		}
		float rot2 = ldata[maxpos].angle;
        if(abs(rot2) > 1.0) rot2 = 1.0; 
        if(turning == false){
            turning = true;    
            giro    = rand()%2;

            if(br > 10){
                br = 0;
               cout << "BIG ROTATION.." << endl;
               differentialrobot_proxy->setSpeedBase(0, 1.0);
            }
			if(giro == 0){
                // Giro izquierda
                differentialrobot_proxy->setSpeedBase(0, rot2);
                br++;
            } else{
                // Giro derecha
                if(abs(rot2) <= 0.5)
                     differentialrobot_proxy->setSpeedBase(0, rot2*2);
                else
                    differentialrobot_proxy->setSpeedBase(0, rot2);
		    }
        } else{
            if(giro == 0){
                // Giro izquierda
                differentialrobot_proxy->setSpeedBase(0, rot2);
                br++;
            } else{
                // Giro derecha
                if(abs(rot2) <= 0.5)
                     differentialrobot_proxy->setSpeedBase(0, rot2*2);
                else
                    differentialrobot_proxy->setSpeedBase(0, rot2);
		    }
        }
    }        
}

void SpecificWorker::findObstacle(RoboCompLaser::TLaserData ldata)
{
    static bool turning = false;
    static int giro = 0;
    int maxpos = 0;
    for(int i=0;i<ldata.size(); i++){
        if(abs(ldata[i].angle) > abs(ldata[maxpos].angle))
            maxpos = i;
    }
    float rot2 = ldata[maxpos].angle;
     if(abs(rot2) > 1.0) rot2 = 1.0; 
    if(turning == false){
        turning = true;
        giro    = rand()%2;
        if(giro == 0){
            // Giro izquierda
            differentialrobot_proxy->setSpeedBase(0, rot2);

            setState(SpecificWorker::State::walk);
        } else{
            // Giro derecha
            differentialrobot_proxy->setSpeedBase(0, rot2);

            setState(SpecificWorker::State::walk);
        }
    } else{
        if(giro == 0){
            // Giro izquierda
            differentialrobot_proxy->setSpeedBase(0, rot2);

            setState(SpecificWorker::State::walk);

        } else{
            // Giro derecha
            differentialrobot_proxy->setSpeedBase(0, rot2);

            setState(SpecificWorker::State::walk);
        }
    }
}





///////////////////////////////////////////////////////////////////77
////  SUBSCRIPTION
/////////////////////////////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const Pick &myPick)
{
//subscribesToCODE

}



