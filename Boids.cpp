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

bool Boids::proximity(Agent a1, Agent a2)
{
  double d = sqrt((a2.x-a1.x)*(a2.x-a1.x)+(a2.y-a1.y)*(a2.y-a1.y)); // Calcul si a2 est dans le rayon de perception de a1
  if(d<Agent::R)
    { 
      return true;
    }
  return false;
}

int Boids::neighbours(int p)
{
  int i,K=0;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i]))
       {
         K++;
       }
    }
  K--; // on enlève l'élément comparé à lui même
  return K;
}

double * Boids::v1(int p)
{
  double * v1 = new double[2];
  v1[0]=0;
  v1[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i])) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v1[0] += (data[i].vx-data[p].vx); // On ajoute la différence à v1
	  v1[1] += (data[i].vy-data[p].vy);
	}
    }
  v1[0] /= neighbours(p); // On divise par le nb d'agents dans le rayon de perception
  v1[1] /= neighbours(p);

  return v1;
}

double * Boids::v2(int p)
{
  double * v2 = new double[2];
  v2[0]=0;
  v2[1]=0;
  int i;
  for(i=0;i<nb_Agents;i++)
    {
      if(proximity(data[p],data[i])) // Si l'élément est dans le rayon de perception de data[p]
	{
	  v2[0] += (data[i].x-data[p].x); // On ajoute la différence à v2
	  v2[1] += (data[i].y-data[p].y);
	}
    }
  v2[0] /= neighbours(p); // On divise par le nb d'agents dans le rayon de perception
  v2[1] /= neighbours(p);

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
