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

double * Boids::v1(int p)
{
  double * v1 = new double[2];
  int * voisins = neighbours(p,Agent::R);
  v1[0]=0;
  v1[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i],Agent::R) && !data[i].isObstacle) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v1[0] += (data[i].vx-data[p].vx); // On ajoute la différence à v1
	  v1[1] += (data[i].vy-data[p].vy);
	}
    }
  v1[0] /= voisins[0]; // On divise par le nb d'agents dans le rayon de perception
  v1[1] /= voisins[0];
	
  delete [] voisins;	
  return v1;
}

double * Boids::v2(int p)
{
  double * v2 = new double[2];
  int * voisins = neighbours(p,Agent::R);
  v2[0]=0;
  v2[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i],Agent::R) && !data[i].isObstacle) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v2[0] += (data[i].x-data[p].x); // On ajoute la différence à v2
	  v2[1] += (data[i].y-data[p].y);
	}
    }
  v2[0] /= voisins[0]; // On divise par le nb d'agents dans le rayon de perception
  v2[1] /= voisins[0];

  delete [] voisins;
  return v2;
}

double * Boids::v3(Agent a)
{
  
}

void Boids::v(Agent a)
{
  
}
// ===========================================================================
//                                Protected Methods
// ===========================================================================

// ===========================================================================
//                               Non inline accessors
// ===========================================================================
