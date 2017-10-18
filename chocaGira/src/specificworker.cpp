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
#include "specificworker.h"
#include <qt4/QtCore/qdebug.h>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	
}

float SpecificWorker::f1(float d)
{
  return (1/(1+exp(-d)-0.5));
}
float SpecificWorker::f2(float r,float h, float Vx)
{
  float y;
  
  y=(-pow(Vx,2))/log(h);
  return exp((-pow(r,2))/y);
  
}


bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

	innerModel = new InnerModel("/home/ruben/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
try{
	  
	  RoboCompDifferentialRobot::TBaseState bState;
	  differentialrobot_proxy->getBaseState (bState );
	  innerModel->updateTransformValues ("base",bState.x, 0, bState.z, 0, bState.alpha, 0 );
	  QVec ini;
	  float angulo;
	  float distanciaObjetivo;
	  const float MAXADV = 400;
 	  const float MAXROT=0.5;
	  
	  //si no hay target no hago nada
	  //si hay, corrijo la velocidad del robot
	  //si he llegado, elimino el target	
	  //si se ha seleccionado un objetivo
	  
	  
	  if(pick.isActive()){
  
	    
	    QVec tr = innerModel->transform ("base",pick.getAux(),"world" );
	    angulo = atan2 ( tr.x(),tr.z());
	    distanciaObjetivo = tr.norm2();
	    
	    
	    
 	    if ( distanciaObjetivo < 50 )
 	    {
 		pick.setActive (false);
 		differentialrobot_proxy->setSpeedBase(0,0);
 	    }else
 	    {
	      float vAdv = distanciaObjetivo;
 	      float vrot = angulo;
 	      
 	      if(vrot > MAXROT)
 		vrot=MAXROT;
	      
	      if(vrot< -MAXROT)
		vrot=-MAXROT;
	      
	    vAdv = MAXADV*f1(vAdv)*f2(vrot,0.9,0.1);
	    differentialrobot_proxy->setSpeedBase(vAdv,vrot);
	    
	      
	    }

	}
	}catch ( const Ice::Exception &ex ){
			std::cout << ex << std::endl;
	}
  
}


void SpecificWorker::setPick(const Pick &myPick)
{

    qDebug() << "New target selected: " << myPick.x << myPick.z;
    pick.copy(myPick.x, myPick.z);
    pick.setActive(true);
  
}







