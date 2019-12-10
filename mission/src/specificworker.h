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

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	bool GotoPoint_atTarget();
	void GotoPoint_go(string nodo, float x, float y, float alpha);
	void GotoPoint_stop();
	void GotoPoint_turn(float speed);
	void RCISMousePicker_setPick(Pick myPick);

	RoboCompGenericBase::TBaseState bState;
	RoboCompLaser::TLaserData ldata;
	struct Coords
	{
		QMutex mutex;
		std::atomic<bool> active = false;
		int x,z, alpha;

		void getCoords(int &x_, int &z_, int &alpha_){
			QMutexLocker ml(&mutex);
			x_ = x;
			z_ = z;
			alpha_ = alpha;
		}
		void setCoords(int x_, int z_, int alpha_){
			QMutexLocker ml(&mutex);
			x = x_;
			z = z_;
			alpha = alpha_;
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

};

#endif
