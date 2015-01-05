//****************************************************************************
//
//
//
//****************************************************************************



#ifndef __BOIDS_H__
#define __BOIDS_H__


// ===========================================================================
//                                  Libraries
// ===========================================================================
#include <cstdio>
#include <cstdlib>
#include <typeinfo>
#include "Agent.h"


// ===========================================================================
//                                Project Files
// ===========================================================================




// ===========================================================================
//                              Class declarations
// ===========================================================================






class Boids
{
  public :
    
    // =======================================================================
    //                                 Enums
    // =======================================================================
    
    // =======================================================================
    //                               Constructors
    // =======================================================================
    Boids(void);

    // =======================================================================
    //                                Destructor
    // =======================================================================
    ~Boids(void);

    // =======================================================================
    //                            Accessors: getters
    // =======================================================================

    // =======================================================================
    //                            Accessors: setters
    // =======================================================================

    // =======================================================================
    //                                Operators
    // =======================================================================

    // =======================================================================
    //                              Public Methods
    // =======================================================================
    void addAgent(Agent a);
    double * v1(int p);
    double * v2(int p);
    double * v3(Agent a);
    void v(Agent a);
    bool proximity(Agent a1, Agent a2, double d); // La distance entre a1 et a2 est elle inférieure à d ?
    int * neighbours(int p,double d); // Retourne le nombre d'agents et d'obstacles dans le rayon d de l'agent a la position p

    // =======================================================================
    //                             Public Attributes
    // =======================================================================
    Agent * data;
    int nb_Agents;




  protected :

    // =======================================================================
    //                            Forbidden Constructors
    // =======================================================================
    /*Boids(void)
    {
      printf("%s:%d: error: call to forbidden constructor.\n", __FILE__, __LINE__);
      exit(EXIT_FAILURE);
    };
    Boids(const Boids &model)
    {
      printf("%s:%d: error: call to forbidden constructor.\n", __FILE__, __LINE__);
      exit(EXIT_FAILURE);
      };*/


    // =======================================================================
    //                              Protected Methods
    // =======================================================================

    // =======================================================================
    //                             Protected Attributes
    // =======================================================================
};


// ===========================================================================
//                              Getters' definitions
// ===========================================================================

// ===========================================================================
//                              Setters' definitions
// ===========================================================================

// ===========================================================================
//                             Operators' definitions
// ===========================================================================

// ===========================================================================
//                          Inline functions' definition
// ===========================================================================


#endif // __BOIDS_H__

