//****************************************************************************
//
//
//
//****************************************************************************


 
 
// ===========================================================================
//                                   Libraries
// ===========================================================================
#include <cstdio>
#include <math.h>
// ===========================================================================
//                                 Project Files
// ===========================================================================
#include "Boids.h"
#include "Agent.h"




//############################################################################
//                                                                           #
//                           Class Boids                                     #
//                                                                           #
//############################################################################

// ===========================================================================
//                         Definition of static attributes
// ===========================================================================

// ===========================================================================
//                                  Constructors
// ===========================================================================
Boids::Boids(void)
{
  nb_Agents = 0;
  data = new Agent[50];
}

// ===========================================================================
//                                  Destructor
// ===========================================================================
Boids::~Boids(void)
{
  delete [] data;
}

// ===========================================================================
//                                 Public Methods
// ===========================================================================
void Boids::addAgent(Agent a)
{
  data[nb_Agents] = a;
  nb_Agents++;
}

bool Boids::proximity(Agent a1, Agent a2, double distance)
{
  double d = sqrt((a2.x-a1.x)*(a2.x-a1.x)+(a2.y-a1.y)*(a2.y-a1.y)); // Calcul si a2 est dans le rayon de perception de a1
  if(d<distance)
    { 
      return true;
    }
  return false;
}

int * Boids::neighbours(int p,double d)
{
  int * tab = new int[2];
  int i,a=0, o=0;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i],d))
       {
		 if(!data[i].isObstacle)
			{	  
				a++; 
			}
		 else 
		     {
				 o++; 
			 }
       }
    }
  a--; // on enlève l'agent comparé à lui même
  tab[0]=a;// nombre d'agents
  tab[1]=o;// nombre d'obstacles
  return tab;
}

double * Boids::velocity1(int p)
{
  double * v1 = new double[2];
  int * voisins = neighbours(p,Agent::R);
  v1[0]=0;
  v1[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i],Agent::R) && !data[i].isObstacle && voisins[0]!=0) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v1[0] += (data[i].vx-data[p].vx)/voisins[0]; // On ajoute la différence à v1
	  v1[1] += (data[i].vy-data[p].vy)/voisins[0];
	}
    }
	
  delete [] voisins;	
  return v1;
}

double * Boids::velocity2(int p)
{
  double * v2 = new double[2];
  int * voisins = neighbours(p,Agent::R);
  v2[0]=0;
  v2[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i],Agent::R) && !data[i].isObstacle && voisins[0]!=0) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v2[0] += (data[i].x-data[p].x)/voisins[0]; // On ajoute la différence à v2
	  v2[1] += (data[i].y-data[p].y)/voisins[0];
	}
    }

  delete [] voisins;
  return v2;
}

double * Boids::velocity3(int p)
{
  double * v3 = new double[2];
  int * voisins = neighbours(p,Agent::C);
  v3[0]=0;
  v3[1]=0;
  int i;
  double v3xa=0,v3xo=0,v3ya=0,v3yo=0; // variables intérmédiaires pour stocker la somme avant de diviser le tout
  for(i=0;i<nb_Agents;i++)
    {
	  if(proximity(data[p],data[i],Agent::C) && !data[i].isObstacle && voisins[0]!=0)
		{
			v3xa += (data[i].x-data[p].x)/voisins[0];
			v3ya += (data[i].y-data[p].y)/voisins[0];
		}
	  else if(proximity(data[p],data[i],Agent::C) && data[i].isObstacle && voisins[1]!=0)
	    {
			v3xo += (data[i].x-data[p].x)/voisins[1];
			v3yo += (data[i].y-data[p].y)/voisins[1];
	    }
	}
  v3[0]= - v3xa - v3xo;
  v3[1]= - v3ya - v3yo;
  
  return v3;
}

void Boids::velocity(double gamma1 , double gamma2 , double gamma3, double dt)
{
   int i;
   double * v1;
   double * v2;
   double * v3;
   for(i=0;i<nb_Agents;i++)
     {
		 if(!data[i].isObstacle)
			{
				v1 = velocity1(i);
				v2 = velocity2(i);
				v3 = velocity3(i);
				
				data[i].vx += dt*gamma1*v1[0] + gamma2*v2[0] + gamma3*v3[0];
				data[i].vy += dt*gamma1*v1[1] + gamma2*v2[1] + gamma3*v3[1];
				//printf("%d : v1x = %f v2x = %f v3x = %f vx= %f\n",i,v1[0],v2[0],v3[0],data[i].vx); 
				/* ça ne marche pas car quand on passe à a2, pour calculer la v1 de a2, on utilise le nouveau vx
				 * et vy de a1 ! Faut il calculer avec l'ancien vx/vy de a1 ou le nouveau ? A demander !
				 * */
			}
     } 
}

void Boids::position(double gamma1 , double gamma2 , double gamma3, double dt)
{
  int i;
  velocity(gamma1,gamma2,gamma3,dt);
  for(i=0;i<nb_Agents;i++)
    {
		if(!data[i].isObstacle)
		  {
			  data[i].x += dt*data[i].vx;
			  data[i].y += dt*data[i].vy;
		  }
	}
}
// ===========================================================================
//                                Protected Methods
// ===========================================================================

// ===========================================================================
//                               Non inline accessors
// ===========================================================================
