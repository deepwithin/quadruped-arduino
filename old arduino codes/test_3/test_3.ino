#include <math.h>
#define pi 3.14159265


void leg_control()
{
  static float i=-157, j=0, flagx=1, flagy=1;
  float x,y,h,k;
  
  if(flagx)
  {
    i++;
    
    x=i/100;
    h=5*sin(x);
    k=0;
    Serial.print("x -> ");
    Serial.print(h);
    Serial.print(" y -> ");
    Serial.println(k);
    
    if(i>157)
      flagx=0;
  }

  else if(!flagx)
  {
    i-=2;

    x=i/100;
    h=5*sin(x);
    Serial.print("x -> ");
    Serial.print(h);

    if(i<-157)
      flagx=1;

      //y part
    if(flagy)
    {
     // flagy=1;
      j+=4;
      if(j>=314)
        flagy=0;
    }

    else if(!flagy)
    {
      j-=4;
     // flagy=0;
     if(j<=0)
        flagy=1;
    }
      
    
    y=j/100;
    k=10*sin(y);
    Serial.print(" y -> ");
    Serial.println(k);
    
  }
}


void setup() {
  Serial.begin(9600);

}

void loop() {
  leg_control();

}
