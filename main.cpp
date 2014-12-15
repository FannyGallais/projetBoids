#include <stdio.h>
#include "bwindow.h"
#include "Agent.h"
#include "Boids.h"


int main()
{

  Agent a1;
  Agent a2;
  printf("%f %f %f %f\n",a1.x,a2.x,a1.y,a2.y);
  Boids b;
  b.addAgent(a1);
  b.addAgent(a2);
  printf("%d\n",b.neighbours(0));
  
  /*bwindow win(640,480);
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
	win.draw_point(100,100,0xFF00);
	win.draw_line(100,100,200,200,0xFF0000);
	win.draw_text(10,10,0x0,"Hello World",strlen("Hello World"));
	win.draw_square(200,200,220,220,0xFF00);
	win.draw_fsquare(400,400,440,440,0xFF00);
	}*/


    return 0;
}
