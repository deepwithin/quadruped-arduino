#include <math.h>
#define pi 3.14159265
#define deltax 0.01 // delta x&y needs to be same to remain in sync
#define deltay 0.01


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}



void loop() {
  float xi =0;
  double yi =0;
  float theta =0;
  Serial.println("X :");
  Serial.println(xi);
  for (theta;theta<= 90;theta = theta+0.5)
     { xi = 5*sin(theta*pi/180);
       yi = 10*cos(theta*pi/180);
      
       Serial.println("X :");
       Serial.println(xi);
       Serial.println("Y :");
         Serial.println(yi);
     }

    for (theta;theta>=-90;theta = theta-0.5)
     { xi = 5*sin(theta*pi/180);
       yi = 10*cos(theta*pi/180);
      
       Serial.println(xi);
         Serial.println(yi);
     }

    for (theta;theta<=0;theta = theta+0.5)
     { xi = 5*sin(theta*pi/180);
       yi = 10*cos(theta*pi/180);
      
       Serial.println(xi);
         Serial.println(yi);
     }
     
}
