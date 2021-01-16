#include <math.h>
#define pi 3.14
//159265
#define deltax 0.01 // delta x&y needs to be same to remain in sync
#define deltay 0.01


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

float values[9]={0,0,-pi/2,0,0,0,0,0}; // order- 0-alpha,1-beta,2-theta x-x2,3-theta y-y2,4- x, 5-y, 6-flag x, 7-flag y, 8-phase-fi
float x, y, alpha, beta, theta, thetaprime, Ax=10, Ay=7, dx, dy, x2, y2;
//values[2]=-pi/2;
//values[3]=0;

float* legcontrol (float* values, float fix, float fiy)
{
  x2=values[2], y2=values[3];
  static int flagx=1, flagy=1;

  if(((int)(x2*100)) == ((int)(pi*100/2)))
  {  
    flagx = 0;
    dx = deltax*4;
  }
  else if(((int)(x2*100)) == ((int)(-pi*100/2)))
  {  
    flagx = 1;
    dx = deltax*2;
  }

  if(!flagx)
  {
    x = 0 + Ax*sin(x2+fix);
    x2 += dx;
    Serial.println(x2);
  }
  else
  {
    x = 0 + Ax*sin(x2+fix);
    x2 -= dx;
    Serial.println(x2);
  }

  
//------------------------------------------------------  
/*  if(((int)(y2*100)) == ((int)(0)))
  {  
    flagy = 1;
    dy = deltay*2;
  }
  if(((int)(y2*100)) == ((int)(pi*100)))
  {  
    flagy = 0;
//    y = ypush;
  }

  if(flagy && !flagx)
  {
    y = height + Ay*sin(y2+fiy);
    y2 += dy;
  }
  else
  {
    y = height;
  }
*/
  values[4] = x;
  values[5] = y;

  flagx=values[6];
  flagy=values[7];

  values[2]=x2;
  values[3]=y2;

  Serial.println(x2);
  Serial.println(y2);
  Serial.println(x);
  Serial.println(y);

  return values;
}



void loop() {
  // put your main code here, to run repeatedly:
  legcontrol(values,0,0);
}
