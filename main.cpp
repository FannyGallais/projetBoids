#include <stdio.h>
#include "bwindow.h"
#include "Agent.h"
#include "Boids.h"
#include <typeinfo>


int main()
{
  Boids b; 
  
  Agent a1;
  Agent a2;
  Agent o = Agent(true);
  Agent a3;
  Agent a4;
  Agent a5;
  Agent a6;
  Agent a7;
  Agent o2 = Agent(true);
  Agent a8;
  b.addAgent(a1);
  b.addAgent(a2); // Attention lorsqu'on modifie l'agent mis dans le tableau ça ne modifie pas l'agent d'origine
  b.addAgent(o);
  b.addAgent(a3);
  b.addAgent(a4);
  b.addAgent(a5);
  b.addAgent(a6);
  b.addAgent(a7);
  b.addAgent(o2);
  b.addAgent(a8);
  /*printf("position de a1 : x= %f y= %f\n",a1.x,a1.y);
  printf("position de a2 : x= %f y= %f\n",b.data[1].x,b.data[1].y);
  printf("position de o : x= %f y= %f\n",b.data[2].x,b.data[2].y);
  printf("position de a3 : x= %f y= %f\n",b.data[3].x,b.data[3].y);*/
  
  // Test velocity
  /*double * v1 = b.velocity1(0);
  printf("v1x : %f    v1y : %f\n",v1[0],v1[1]); // ça fait 0 c'est normal car on a initialisé les vx et vy à 0
  double * v2 = b.velocity2(0);
  printf("v2x : %f    v2y : %f\n",v2[0],v2[1]);
  double * v3 = b.velocity3(0);
  printf("v3x : %f    v3y : %f\n",v3[0],v3[1]);
  b.velocity(1,2,3,1);
  printf("Hors méthode vx de a1 : %f    vy de a1 : %f\n",b.data[0].vx,b.data[0].vy);*/
  
  // Test position
  /*printf("1er tour\n");
  printf("Position de a1 : x = %f y = %f\n",b.data[0].x,b.data[0].y);
  printf("vx de a1 : %f    vy de a1 : %f\n",b.data[0].vx,b.data[0].vy);
  b.position(1,2,3,1);
  printf("2e tour\n");
  printf("Position de a1 : x = %f y = %f\n",b.data[0].x,b.data[0].y);
  printf("Hors méthode vx de a1 : %f    vy de a1 : %f\n",b.data[0].vx,b.data[0].vy);
  b.position(1,2,3,1);
  printf("3e tour\n");
  printf("Position de a1 : x = %f y = %f\n",b.data[0].x,b.data[0].y);
  printf("vx de a1 : %f    vy de a1 : %f\n",b.data[0].vx,b.data[0].vy);*/
  
  /*delete [] v1;
  delete [] v2;
  delete [] v3;*/

  
  
  
  
  bwindow win(1200,1500);
    printf("%d\n",win.init());
    win.map();
    for(;;)
    {
	int ev = win.parse_event();
	switch(ev)
	{
	    case BKPRESS :
		printf("keypressed\n"); 
		printf("key : %s\n",win.get_lastkey());
		break;
	    case BBPRESS:
		printf("buttonpressed\n"); break;
	    case BEXPOSE:
		printf("expose\n"); break;
	    case BCONFIGURE:
		printf("configure\n"); break;
	}
	
	int i;
	win.draw_fsquare(0,0,1200,1500,0xFEFEFE);
	for(i=0;i<b.nb_Agents;i++)
		{
		  if(!b.data[i].isObstacle)
		    {
			  win.draw_fsquare(b.data[i].x-5,b.data[i].y-5,b.data[i].x+5,b.data[i].y+5,0xEF009F);
		    } 
		   else 
			{
		      win.draw_fsquare(b.data[i].x-5,b.data[i].y-5,b.data[i].x+5,b.data[i].y+5,0x000000);
			}
			
		}
    usleep(10000);
	b.position(0.1,0.1,0.1,0.05);
	
	}


    return 0;
}
