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
#include "Predateur.h"




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

Boids::Boids(int size)
{
  nb_Agents = 0;
  data = new Agent[size];
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
		 if(!data[i].isObstacle && !data[i].isPredateur)
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
  double norm = sqrt(v1[0]*v1[0]+v1[1]*v1[1]);	
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

double * Boids::velocity4(int p)
{
  int i=0;
  double xp=0,yp=0; // Coordonnées du prédateurs
  double * v4 = new double[2];
  v4[0]=0;
  v4[1]=0;

  for(i=0;i<nb_Agents;i++)
  {
	  if(data[i].isPredateur && p!=i)
	  {
		xp = data[i].x;
		yp = data[i].y;
		if(sqrt((data[p].x-xp)*(data[p].x-xp)+(data[p].y-yp)*(data[p].y-yp))<Agent::R)
		{
		  v4[0]= -(xp-data[p].x)/(sqrt((xp-data[p].x)*(xp-data[p].x) + (yp-data[p].y)*(yp-data[p].y)));
	      v4[1]= -(yp-data[p].y)/(sqrt((xp-data[p].x)*(xp-data[p].x) + (yp-data[p].y)*(yp-data[p].y)));
	    }
	  }	
  }
  return v4;
}


void Boids::velocity(double gamma1, double gamma2, double gamma3, double gamma4, double dt)
{

  int i;
  double * v1;
  double * v2;
  double * v3;
  double * v4;
  double k = 1;
  for(i=0; i<nb_Agents ; i++)
  {
          if(!data[i].isObstacle)
          {
            v1 = velocity1(i);
            v2 = velocity2(i);
            v3 = velocity3(i);
            v4 = velocity4(i);
            
            if(data[i].y<50)
            {
             //data[i].vx += dt*(gamma1*v1[0] + gamma2*v2[0] + gamma3*v3[0] + gamma4*v4[0]);
             data[i].vy = data[i].vy + k ;
            }
            else if(data[i].x>1200)
            {
             data[i].vx = -data[i].vx - k;
             //data[i].vy += dt*(gamma1*v1[1] + gamma2*v2[1] + gamma3*v3[1] + gamma4*v4[1]);
            }
            else if(data[i].y>900)
            {
             //data[i].vx +=dt*(gamma1*v1[0] + gamma2*v2[0] + gamma3*v3[0] + gamma4*v4[0]);
             data[i].vy = -data[i].vy - k ;
            }
            else if(data[i].x<50)
            {
             data[i].vx = data[i].vx + k;
             //data[i].vy += dt*(gamma1*v1[1] + gamma2*v2[1] + gamma3*v3[1] + gamma4*v4[1]) ;
            }
            else
            {
            
            data[i].vx += dt*(gamma1*v1[0] + gamma2*v2[0] + gamma3*v3[0]+ gamma4*v4[0]);
            data[i].vy += dt*(gamma1*v1[1] + gamma2*v2[1] + gamma3*v3[1]+ gamma4*v4[1]);   //attention problème on calcule les velocity1 2 et 3 à partir des vx et vy précédents
        }
        /*
            //a mattre à la toute fin des calculs de vitesses
            int j=0;
            while(sqrt((data[i].vx*data[i].vx)+(data[i].vy*data[i].vy)>1))
            {
              data[i].vx -= 10;
              data[i].vy -= 10;
              j++;
            }
            * */
            
      }  
  }

}


// Reperage de la proie c a d recherche de l'agent le plus proche dans le rayon Rp du prédateur.
int Boids::reperageProie(int p) // p position du prédateur dans le tableau
{
	int i;
	int proie=nb_Agents; // indice de la proie dans le tableau, s'il n'ya pas de proie il faut que l'indice soit hors du tableau
	double xproie=100,yproie=100;
	double d1 = sqrt((data[p].x-xproie)*(data[p].x-xproie)+(data[p].y-yproie)*(data[p].y-yproie));
	for(i=0; i<nb_Agents;i++)
	{
	  if(!data[i].isObstacle && i!=p && proximity(data[i],data[p],Predateur::Rp))
	  {
		  double d2 = sqrt((data[i].x-data[p].x)*(data[i].x-data[p].x)+(data[i].y-data[p].y)*(data[i].y-data[p].y));
		  if(d2<d1)
		  {
			  d1 = d2;
			  xproie = data[i].x;
			  yproie = data[i].y;
			  proie = i;
		  }
	  }
	}
  return proie;	
}



void Boids::position(double gamma1 , double gamma2 , double gamma3, double gamma4, double dt)
{
  int i;
  velocity(gamma1,gamma2,gamma3, gamma4, dt);
  for(i=0;i<nb_Agents;i++)
    {
		if(!data[i].isObstacle )
		  {
			  data[i].x += dt*data[i].vx;
			  data[i].y += dt*data[i].vy;
		  }
		/*else if(data[i].isPredateur)
		{
			data[i].deplacementAleatoire();
			/*int p = reperageProie(i);
			if(p>=nb_Agents)
			{
				data[i].deplacementAleatoire();
			} else {
				data[i].Chasse(data[p].x,data[p].y);
			}
		}*/
		  
	}
}

// ===========================================================================
//                                Protected Methods
// ===========================================================================

// ===========================================================================
//                               Non inline accessors
// ===========================================================================
