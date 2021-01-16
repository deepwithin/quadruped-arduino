#include <Servo.h>
#include <math.h>

#define f 10.5       //femur length in cm p-17
#define t 13         //tibia length in cm p-18.5
#define pi 3.14159265
#define deltax 0.01 // delta x&y needs to be same to remain in sync
#define deltay 0.01

Servo hlf_srv;
Servo hlb_srv;
Servo hrf_srv;
Servo hrb_srv;

Servo klf_srv;
Servo klb_srv;
Servo krf_srv;
Servo krb_srv;

float height = -18, ypush = -1; // both negative quantities

float values[9]; // order- 0-alpha,1-beta,2-theta x-x2,3-theta y-y2,4- x, 5-y, 6-flag x, 7-flag y, 8-phase-fi
float x, y, alpha, beta, theta, thetaprime, Ax=10, Ay=7, dx, dy, x2, y2;
//values[4] = -pi/2;
//values[5] = {0.0};


void setup()
{
  hlf_srv.attach(3);
  hlb_srv.attach(4);
  hrf_srv.attach(5);
  hrb_srv.attach(6);
  
  klf_srv.attach(7);
  klb_srv.attach(8);
  krf_srv.attach(10);
  krb_srv.attach(11);

  Serial.begin(9600);
}

float* find_alpha (float* values)
{
  float h, k, a, b, D, perpendicular, base, alpha, ratio;
  h=values[4];
  k=values[5];

  a = ((h*h)+(k*k)+(f*f)-(t*t))/2;
  b = (h*h)+(k*k);
  D = (4*k*k*a*a)-(4*b*((a*a)-(f*f*h*h)));

  perpendicular = ((2*k*a)+sqrt(D))/(2*b);
//  Serial.print("perp -> ");
//  Serial.println(perpendicular);

  D = (4*h*h*a*a)-(4*b*((a*a)-(f*f*k*k)));

  base = ((2*h*a)-sqrt(D))/(2*b);
//  Serial.print("base -> ");
//  Serial.println(base);

  ratio = perpendicular/base;

  alpha = (atan2((-perpendicular),(-base))*180/pi);
//  alpha = (atan(ratio)*180/pi);
  values[0]=alpha;
//  Serial.print("alpha : ");
//  Serial.println(alpha);
  
  return values;
}

float* find_beta (float* values)
{
  float x=values[4], y=values[5], alpha=values[0];
  
  float beta, thetaprime, theta1, sine_beta, tanthetap;
//  tanthetap = abs(y)/(-x);
  
  thetaprime = atan2(abs(y),(-x));
//  Serial.println(thetaprime);
/*  if(x==0)
    thetaprime = pi/2;*/
  theta1 = thetaprime - (alpha*pi/180);
  sine_beta = abs(y) * sin(theta1)/(t * sin(thetaprime));
//  Serial.println(sine_beta);
  beta = (asin(sine_beta)*180/pi);
//  values[0] = alpha;
  values[1] = beta;
//  Serial.print("beta : ");
//  Serial.println(beta);

  return values;
}

float* legcontrol(float* values, float fix, float fiy)
{
  static float i=-157, j=0, flagx=1, flagy=1;
  float x,y,h,k;
  
  if(flagx)
  {
    i+=4;
    
    x=i/100;
    h=5*sin(x);
    k= height + 0;
    Serial.print("x -> ");
//    Serial.print("(");
    Serial.print(h);
    Serial.print(" y -> ");
//    Serial.print(",");
    Serial.println(k);
//    Serial.print(") ");
    
    if(i>157)
      flagx=0;
  }

  else if(!flagx)
  {
    i-=4;

    x=i/100;
    h=5*sin(x);
    Serial.print("x -> ");
//    Serial.print("(");
    Serial.print(h);

    if(i<-157)
      flagx=1;

      //y part
    if(flagy)
    {
     // flagy=1;
      j+=8;
      if(j>=314)
        flagy=0;
    }

    else if(!flagy)
    {
      j-=8;
     // flagy=0;
     if(j<=0)
        flagy=1;
    }
      
    
    y=j/100;
    k= height + 5*sin(y);
    Serial.print(" y -> ");
//    Serial.print(",");
    Serial.println(k);
//    Serial.print(") ");
    
  }
  values[4]=h;
  values[5]=k;
}

/*
float* legcontrol (float* values, float fix, float fiy)
{
  x2=values[2], y2=values[3];
  static int flagx, flagy;

  if(((int)(x2*100)) == ((int)(pi*100/2)))
  {  
    flagx = 0;
    dx = deltax*4;
  }
  if(((int)(x2*100)) == ((int)(-pi*100/2)))
  {  
    flagx = 1;
    dx = deltax*2;
  }

  if(!flagx)
  {
    x = 0 + Ax*sin(x2+fix);
    x2 += dx;
  }
  else
  {
    x = 0 + Ax*sin(x2+fix);
    x2 -= dx;
  }
  
//------------------------------------------------------  
  if(((int)(y2*100)) == ((int)(0)))
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

  values[4] = x;
  values[5] = y;

  flagx=values[6];
  flagy=values[7];

  values[2]=x2;
  values[3]=y2;

  Serial.println(x);
  Serial.println(y);

  return values;
}
*/

void horsewalk(float* values)
{
  
//1
/*  legcontrol(values,pi/2,0);
  find_alpha(values);
  find_beta(values);
  hlb_srv.write(values[0]);
  klb_srv.write(values[1]);
//2
  legcontrol(values,pi/6,0);
  find_alpha(values);
  find_beta(values);  
  hlf_srv.write(values[0]);
  klf_srv.write(values[1]);*/
//3
  legcontrol(values,0,0);
  find_alpha(values);
  find_beta(values);
  Serial.print("alpha : ");
  Serial.println((values[0]));
  Serial.print("beta : ");
  Serial.println(values[1]);
//  delay(100);
  hrb_srv.write(values[0]+90);
  krb_srv.write(180-values[1]);
//4
/*  legcontrol(values,pi/2,pi/2);
  find_alpha(values);
  find_beta(values);
  hrf_srv.write(values[0]);
  krf_srv.write(values[1]);*/

}

/*
float* legcontrol2(float* values, float fix, float fiy)
{

  if(((int)(x2*100)) == ((int)(pi*100/2)))
  {  
    flagx = 0;// upar uth ke aage jaa rha hai
    dx = deltax*4;
  }
  if(((int)(x2*100)) == ((int)(-pi*100/2)))
  {  
    flagx = 1;// zammen pe peeche jaa rha hai
    dx = deltax*2;
  }

  if(flagx)
  {
    x = 0 + Ax*sin(x2+fix);
    x2 += dx;
  }
  else
  {
    x = 0 + Ax*sin(x2+fix);
    x2 -= dx;
  }
//_______________________________________________________

  if(!flagx)
  {
    y = height + Ay*sin(y2+fiy);
    y2 += dy;
  }
  else
  {
    y = height;
  }

  values[4] = x;
  values[5] = y;

  flagx=values[6];
  flagy=values[7];

  return values;
  
}
*/

void loop() 
{
  horsewalk(values);
  
}
