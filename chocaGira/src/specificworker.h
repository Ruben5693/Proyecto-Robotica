/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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
	SpecificWorker(MapPrx& mprx);	
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	float f1(float d);
	float f2(float r,float h, float Vx);

public slots:
	void compute(); 	

private:
  
	enum class State {IDLE,GOTO,BUG};
  
	struct Target
	{
	  bool active=false;
 	  mutable QMutex mutex;
	  QVec aux = QVec::zeros(3);
	  
	  bool isActive(){
	    return active;
	  }
	  
	  void setActive(bool act){
	    QMutexLocker lm(&mutex);
	    active = act;
	  }
	  
	  void copy(float x, float z){
 	    QMutexLocker lm(&mutex);
	    aux.setItem(0,x);
	    aux.setItem(1,0);
	    aux.setItem(2,z);
	  }
	  
	  QVec getAux(){
 	    QMutexLocker lm(&mutex);
	    return aux;
	  }
	  
	};
	
	InnerModel* innerModel;
	Target pick;
	State state = State::IDLE;
	void gotoTarget(const TLaserData &tLaser);
	bool obstacle(TLaserData tLaser);
	bool targetAtSight(TLaserData tLaser);
// 	void bug(const TLaserData &tLaser, const TBaseState& bState);
	void bug(const TLaserData &tLaser, const TBaseState &bState);
	//1.-Se dibuja una línea recta “linea” desde la base del robot hacia la meta.
	//2.- El robot sigue esa línea “linea” hasta llegar a una de las siguientes situaciones:
	//	a.-El robot llega a la meta y se para.
	//	b.-El robot llega a un obstáculo, y lo rodea realizando pequeñas rotaciones
	//	y avances siempre hacia la izquierda, hasta encontrase de nuevo con la línea “linea”.
	//	Y se pasa de nuevo al paso número 2
	QLine2D linea; 
// 	float obstacleLeft( const TLaserData& tLaser);
	float obstacleLeft( const TLaserData &tLaser);
	//Almacena la distanciaAnterior distancia en perpendicular hasta la linea
	float distanciaAnterior;
// 	float distanceToLine(const TBaseState& bState);
	float distanceToLine(const TBaseState &bState);

  
};

#endif
