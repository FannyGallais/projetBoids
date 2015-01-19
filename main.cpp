#include <stdio.h>
#include "bwindow.h"
#include "Agent.h"
#include "Boids.h"
#include "Predateur.h"
#include <typeinfo>


int main()
{
  Boids b; 
  Agent o = Agent(true);
  Agent o2 = Agent(true);
  Agent o3 = Agent(true);
  b.addAgent(o);
  b.addAgent(o2);
  b.addAgent(o3);

  bwindow win(1500,1500);
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
	win.draw_fsquare(0,0,1500,1500,0xFEFEFE);
	for(i=0;i<b.nb_Agents;i++)
		{
		  if(b.data[i].isObstacle)
		    {
			  win.draw_fsquare(b.data[i].x-4,b.data[i].y-4,b.data[i].x+4,b.data[i].y+4,0x000000);
			  
		    } 
			else 
			{
				win.draw_fsquare(b.data[i].x-2,b.data[i].y-2,b.data[i].x+2,b.data[i].y+2,0xEF009F);
			}
		}
	win.draw_fsquare(b.dataP[0].x-5,b.dataP[0].y-5,b.dataP[0].x+5,b.dataP[0].y+5,0x03DBCC); 
    usleep(8000);
	b.position(0.1,0.1,1.5,1.5,0.1);
	
	}


    return 0;
}
