//****************************************************************************
//
//
//
//****************************************************************************


 
 
// ===========================================================================
//                                   Libraries
// ===========================================================================



// ===========================================================================
//                                 Project Files
// ===========================================================================
#include "Agent.h"
#include <math.h>



//############################################################################
//                                                                           #
//                           Class Agent                                     #
//                                                                           #
//############################################################################

// ===========================================================================
//                         Definition of static attributes
// ===========================================================================
const double Agent::R = 200;
const double Agent::C = 30;
// ===========================================================================
//                                  Constructors
// ===========================================================================
Agent::Agent(void)
{
  x=(rand()/(double)RAND_MAX)*(640+100)+100;
  y=(rand()/(double)RAND_MAX)*(480+100)+100;
  vx=10;
  vy=-10;
  isObstacle=false;
  isPredateur=false;
}

Agent::Agent(bool b)
{
  x=(rand()/(double)RAND_MAX)*(640+100)+100;
  y=(rand()/(double)RAND_MAX)*(480+100)+100;
  vx=0;
  vy=0;
  isObstacle=b;
  isPredateur=false;
}

// ===========================================================================
//                                  Destructor
// ===========================================================================
Agent::~Agent(void)
{
}

void Agent::deplacementAleatoire(void)
{
  vx=(rand()/(double)RAND_MAX)*15;
  vy=(rand()/(double)RAND_MAX)*15;
  x += vx;
  y += vy;
}

void Agent::Chasse(double xp, double yp) // (xp,yp) les coordonnées de la proie
{
	vx= (xp-x)/(sqrt((xp-x)*(xp-x)+(yp-y)*(yp-y)));
	vy= (yp-y)/(sqrt((xp-x)*(xp-x)+(yp-y)*(yp-y)));
	x += vx;
	y += vy;
}

void Agent::Digestion()
{
	vx=0;
	vy=0;
}
// ===========================================================================
//                                 Public Methods
// ===========================================================================

// ===========================================================================
//                                Protected Methods
// ===========================================================================

// ===========================================================================
//                               Non inline accessors
// ===========================================================================
