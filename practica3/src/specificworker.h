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

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <stdlib.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	enum class State {idle, walk, goTo, skirt};
	SpecificWorker::State actual_state;
	RoboCompGenericBase::TBaseState bState;
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void RCISMousePicker_setPick(Pick myPick);

	struct Coords
	{
		Pick pick;
		QMutex mutex;
		std::atomic<bool> active = false;

		Pick getCoords(){
			QMutexLocker ml(&mutex);
			return pick;
		}
		void setCoords(Pick p){
			QMutexLocker ml(&mutex);
			pick = p;
			active.store(true);
		}
		bool isActive()
		{
			//active=true si tenemos pick
			return active.load();
		}
	};
	Coords c;


public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innerModel;
	void idle();
	void walk();
	void goTo();
	void skirt();
};

#endif
