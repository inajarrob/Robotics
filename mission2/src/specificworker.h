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
#include <Qt>
#include <QVector>
#include <list>

	


class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void AprilTags_newAprilTag(tagsList tags);
	void AprilTags_newAprilTagAndPose(tagsList tags, RoboCompGenericBase::TBaseState bState, RoboCompJointMotor::MotorStateMap hState);

	enum class State {idle, turn, check_target, focus, moveArm};
	SpecificWorker::State actual_state;

	float zero = 0;
	
	using Tp = std::tuple<int, float, float, float, std::string>;
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
		std::tuple<bool, Tp> readSelected(const int &id)
		{
			QMutexLocker ml(&mutex);
			auto r = std::find_if(std::begin(datos), std::end(datos), [id](const auto &tag){ return id == std::get<int>(tag);});
			return std::make_tuple(r != std::end(datos), *r);
		} 
	};

	visited visitedTags, handTags;
	float increment = 0.1;
	std::list<int> visitedIds;

	struct Pose6D
{
 ::Ice::Float x;
 ::Ice::Float y;
 ::Ice::Float z;
 ::Ice::Float rx;
 ::Ice::Float ry;
 ::Ice::Float rz;

 bool operator==(const Pose6D& __rhs) const
 {
 if(this == &__rhs)
 {
 return true;
 }
 if(x != __rhs.x)
 {
 return false;
 }
 if(y != __rhs.y)
 {
 return false;
 }
 if(z != __rhs.z)
 {
 return false;
 }
 if(rx != __rhs.rx)
 {
 return false;
 }
 if(ry != __rhs.ry)
 {
 return false;
 }
 if(rz != __rhs.rz)
 {
 return false;
 }
 return true;
 }

 bool operator<(const Pose6D& __rhs) const
 {
 if(this == &__rhs)
 {
 return false;
 }
 if(x < __rhs.x)
 {
 return true;
 }
 else if(__rhs.x < x)
 {
 return false;
 }
 if(y < __rhs.y)
 {
 return true;
 }
 else if(__rhs.y < y)
 {
 return false;
 }
 if(z < __rhs.z)
 {
 return true;
 }
 else if(__rhs.z < z)
 {
 return false;
 }
 if(rx < __rhs.rx)
 {
 return true;
 }
 else if(__rhs.rx < rx)
 {
 return false;
 }
 if(ry < __rhs.ry)
 {
 return true;
 }
 else if(__rhs.ry < ry)
 {
 return false;
 }
 if(rz < __rhs.rz)
 {
 return true;
 }
 else if(__rhs.rz < rz)
 {
 return false;
 }
 return false;
 }

 bool operator!=(const Pose6D& __rhs) const
 {
 return !operator==(__rhs);
 }
 bool operator<=(const Pose6D& __rhs) const
 {
 return operator<(__rhs) || operator==(__rhs);
 }
 bool operator>(const Pose6D& __rhs) const
 {
 return !operator<(__rhs) && !operator==(__rhs);
 }
 bool operator>=(const Pose6D& __rhs) const
 {
 return !operator<(__rhs);
 }
};


public slots:
	void compute();
	void initialize(int period);

private:
	std::shared_ptr<InnerModel> innerModel;
	void idle();
	void check_target();
	void turn();
	void focus();
	void moveArm();
	int current_id = 0;
};

#endif
