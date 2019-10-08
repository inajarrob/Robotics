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
#include <grid.h>
    const float threshold = 200; // millimeters


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	enum class State {idle, walk, turn, findObj, spiral};
	State actual_state;
	/// Grid
	struct TCell
	{
		uint id;
		bool free;
		bool visited;
		//QGraphicsRectItem* rect;
		float cost = 1;
		
		// method to save the value
		void save(std::ostream &os) const {	os << free << " " << visited; };
		void read(std::istream &is) {	is >> free >> visited ;};
	};
		
	using TDim = Grid<TCell>::Dimensions;
	Grid<TCell> grid;


	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setState(enum State a_state);
	enum State getState();


public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innerModel;
	void walk(RoboCompLaser::TLaserData ldata, enum State a_state);
	void findObstacles(RoboCompLaser::TLaserData ldata, enum State a_state);

};

#endif
