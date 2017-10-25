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
#include <qt4/QtGui/qpolygon.h>

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

	innerModel = new InnerModel("/home/pablo/robocomp/files/innermodel/simpleworld.xml");
	timer.start(Period);
	

	return true;
}

void SpecificWorker::compute()
{
try{
	RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();
	RoboCompDifferentialRobot::TBaseState bState;
	differentialrobot_proxy->getBaseState(bState);
	innerModel->updateTransformValues ("base",bState.x, 0, bState.z, 0, bState.alpha, 0 ); 
	//Vector auxiliar 3D para para obtener la posicion del robot y pasarselas a linea junto con el destino
	QVec auxLinea;
	  
	  
	  switch( state )
	    {
	      case State::IDLE:
		if ( pick.isActive() ){
		  auxLinea = QVec::vec3(bState.x, 0, bState.z);
// 		  línea recta “linea” desde la base del robot hacia la meta.
		  linea = QLine2D( auxLinea, pick.getAux() );
		  state = State::GOTO;
		}
		break;
	      case State::GOTO:
		gotoTarget(laserData);
		break;
	      case State::BUG:
		bug(laserData, bState);
		break;
	    }
	    


  }catch ( const Ice::Exception &ex ){
			std::cout << ex << std::endl;
	}
}

void SpecificWorker::gotoTarget(const TLaserData &tLaser){
  
  if( obstacle(tLaser) == true)   // si hay un obstaculo delante va a BUG
  {
    qDebug() << "Obstaculo detectado, de GOTO a BUG";
    state = State::BUG;
    return;
  }
  
  
  QVec tr = innerModel->transform ("base",pick.getAux(),"world" );
  const float MAXADV = 400;
  const float MAXROT=0.5;
  float angulo;
  float distanciaObjetivo;
  distanciaObjetivo = tr.norm2();
  angulo = atan2 ( tr.x(),tr.z());
  
  if ( distanciaObjetivo < 50 ){
    
    state = State::IDLE;
    pick.setActive (false);
    differentialrobot_proxy->setSpeedBase(0,0);
    qDebug() << "Fin: de GOTO a IDLE";
    return;
    
  }else{
    
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

void SpecificWorker::setPick(const Pick &myPick)
{

    qDebug() << "Nuevo objetivo seleccionado: " << myPick.x << myPick.z;
    pick.copy(myPick.x, myPick.z);
    pick.setActive(true);
  
}


// void SpecificWorker::bug(const TLaserData &tLaser, const TBaseState& bState)
void SpecificWorker::bug(const TLaserData &tLaser, const TBaseState& bState)
{
  
//   qDebug() << "Dentro de Bug";
  if( obstacle(tLaser) == false){
    
    const float alpha = log ( 0.1 ) /log ( 0.3 ); //amortigua /corte
    float distanciaObstaculo = obstacleLeft(tLaser);
    float diffToline = distanceToLine(bState);
      
    if (targetAtSight(tLaser)){
      state = State::GOTO;
      qDebug() << "Objetivo visible: de BUG A GOTO";
      return;
    }      
    
    //El robot llega a un obstáculo, y lo rodea realizando pequeñas rotaciones
    //y avances siempre hacia la izquierda, hasta encontrase de nuevo con la línea “linea”.
    if (distanciaAnterior < 100 and diffToline < 0){
      state = State::GOTO;
      qDebug() << "Cruzando la linea : de BUG a GOTO";
      return;
    }
    
//     differentialrobot_proxy->setSpeedBase(300, 0);
//     usleep(rand()%(1000000-100000+1) + 100000);
//     state=State::GOTO;
      
    float k=0.1;  // pendiente de la sigmoide
    float vrot =  -((1./(1. + exp(-k*(distanciaObstaculo - 450.))))-1./2.);		//sigmoide para meter vrot entre -0.5 y 0.5. La k ajusta la pendiente.
    float vadv = 350 * exp ( - ( fabs ( vrot ) * alpha ) ); 		//gaussiana para amortiguar la vel. de avance en funcion de vrot
    qDebug() << vrot << vadv;
	//vrot *= 0.3;
    differentialrobot_proxy->setSpeedBase ( vadv ,vrot );
    
  }else{

    
      try{
	differentialrobot_proxy->setSpeedBase(0, 0.3);
      }catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
  }
  
}

bool SpecificWorker::obstacle ( TLaserData tLaser ){
  
  const int offset = 35;
  const int minDist = 350;
	
  //ordena los datos del laser desde la distancia mas corta a la mas larga y checkea si el primero
  //es menor que minDist 	 
  std::sort ( tLaser.begin() + offset, tLaser.end()- offset, [] ( RoboCompLaser::TData a, RoboCompLaser::TData b ){return a.dist < b.dist;});
  return ( tLaser[offset].dist < minDist );

}

bool SpecificWorker::targetAtSight(TLaserData tLaser){
  
  QPolygonF polygon;
  for ( auto l: tLaser){
     QVec lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
     polygon << QPointF(lr.x(), lr.z());
  }
  QVec t = pick.getAux();
  return  polygon.containsPoint(QPointF(t.x(), t.z() ),Qt::WindingFill);
  
}


//Devuelve el obstaculo  visible en el laser que esta mas cercano a la izquierda
// El robot llega a un obstáculo, y lo rodea realizando pequeñas rotaciones
//y avances siempre hacia la izquierda
// float SpecificWorker::obstacleLeft(const TLaserData& tLaser){
  float SpecificWorker::obstacleLeft(const TLaserData &tLaser){
  
  const int laserpos = 85;
  float min = tLaser[laserpos].dist;
  for(int i=laserpos-2; i<laserpos+2;i++)
    {
      if (tLaser[i].dist < min)
	min = tLaser[i].dist;
    }
    return min;
}

//Devuelve la diferencia entre la distanciaAnterior a la linea y la distanciaActual a la linea
//si la distanciaAnterior<100 y la diferencia es < 0 estamos cruzando la linea, es decir
//si la distanciaActual a la linea es menor que 100 y la distanciaAnterior>distanciaActual
//hemos llegado a la linea y seguimos hacia el objetivo.
// float SpecificWorker::distanceToLine(const TBaseState& bState){
float SpecificWorker::distanceToLine(const TBaseState &bState){
  
  QVec posicion = QVec::vec3(bState.x, 0., bState.z);
  float distanciaActual = fabs(linea.perpendicularDistanceToPoint(posicion));
  float diferencia = distanciaActual - distanciaAnterior;
  distanciaAnterior = distanciaActual;
  
  return diferencia;
  
}

	






