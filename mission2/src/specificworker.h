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
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void AprilTags_newAprilTag(tagsList tags);
	void AprilTags_newAprilTagAndPose(tagsList tags, RoboCompGenericBase::TBaseState bState, RoboCompJointMotor::MotorStateMap hState);

	enum class State {idle, turn, check_target};
	SpecificWorker::State actual_state;
	
	using Tp = std::tuple<int, float, float, float>;
	struct visited 
	{
		QMutex mutex;
		std::vector<Tp> datos;

		void write(std::vector<Tp> &d)
		{
			QMutexLocker ml(&mutex);
			datos.swap(d);
		}

		std::vector<Tp> read()
		{
			QMutexLocker ml(&mutex);
			return datos;
		}
		/* Td readSelected(const std::string id)
		{
			return std::find(std::begin(datos), std::end(datos), [id](const auto &t){ return t.cameraId == id;});
		} */
	};
	visited visitedTags, handTags;

public slots:
	void compute();
	void initialize(int period);
private:
	std::shared_ptr<InnerModel> innerModel;
	void idle();
	void check_target();
	void turn();
};

#endif
