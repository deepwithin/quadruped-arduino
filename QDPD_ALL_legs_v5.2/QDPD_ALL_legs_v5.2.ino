/*  LEG ARRANGEMENT ASSUMED
 *  quadruped front
 *  
 *  lf 2\     4 rf
 *     | \    |
 *     |  \   |
 *     |   \  |
 *     |    \ |
 *  lb 1     \3 rb
 *  
 *  quadruped back
 *  
 *  ABBREVIATIONS: h-hip, k-knee, l-left, r-right, f-front, b-back
 */

#include <Servo.h>
#include <math.h>

#define f 10.5    //femur length in cm
#define t 13      //tibia length in cm
#define pi 3.14159265

Servo hlf_srv;
Servo hlb_srv;
Servo hrf_srv;
Servo hrb_srv;

Servo klf_srv;
Servo klb_srv;
Servo krf_srv;
Servo krb_srv;

float height = -17.5;   //height of bot from ground- a negative quantity
float values1[4];
float values2[4];
float values3[4];
float values4[4];
float Ax=5 ,Ay=9 , steps=5;
int del=500;                    //delay-value in microseconds
/* amplitude along x
 * amplitude along y
 * steps means speed or the step it will take in every iteration
 */

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
//_________________________________________________________________________________________________________________________________
float* legcontrol2(float* values, float phase)
{
  static float i=-157, j=0, flagx=1, flagy=1;
  float x,y,h,k;
  
  if(flagx)
  {
    i+=steps;
    
    x=i/100;
    h=Ax*sin(x+phase); //_________________________
    
    k= height + 0;
//.............................
///    Serial.print("x -> ");
///    Serial.print(h);
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
    if((i>157) && (i=157))  
      flagx=0;
  }

  else if(!flagx)
  {
    i-=steps;

    x=i/100;
    h=Ax*sin(x+phase); //___________________________
    
//............................
///    Serial.print("x -> ");
///    Serial.print(h);
//............................

    if((i<-157) && (i=-157)) //--------left value not decreased
      flagx=1;
  
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}
//____________________________________________________________________________________________________________________________________
float* legcontrol1(float* values, float phase)
{
  static float i= 154, j=0, flagx=1, flagy=1;  // changed the value of i to create phase difference
  float x,y,h,k;
  
  if(flagx)
  {
    i+=steps;
    
    x=i/100;
    h=Ax*sin(x+phase); //_________________________
    
    k= height + 0;
//.............................
///    Serial.print("x -> ");
///    Serial.print(h);
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
    if((i>157) && (i=157))
      flagx=0;
  }
  else if(!flagx)
  {
    i-=steps;

    x=i/100;
    h=Ax*sin(x+phase); //___________________________
    
//............................
///    Serial.print("x -> ");
///    Serial.print(h);
//............................

    if((i<-157) && (i=-157)) //--------left value not decreased
      flagx=1;

    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}
//_______________________________________________________________________________________________________________________________
float* legcontrol4(float* values, float phase)
{
  static float i=-154, j=0, flagx=1, flagy=1; // i value changed
  float x,y,h,k;
  
  if(flagx)
  {
    i-=steps;
    
    x=i/100;
    h=Ax*sin(x+phase); //_________________________
    
    k= height + 0;
//.............................
///    Serial.print("x -> ");
///    Serial.print(h);
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
    if((i<-157) && (i=-157))
      flagx=0;
  }

  else if(!flagx)
  {
    i+=steps;

    x=i/100;
    h=Ax*sin(x+phase); //___________________________
    
//............................
///    Serial.print("x -> ");
///    Serial.print(h);
//............................

    if((i>157) && (i=157)) //--------left value not decreased
      flagx=1;

    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}
//_________________________________________________________________________________________________________________________________
float* legcontrol3(float* values, float phase)
{
  static float i= -157, j=0, flagx=1, flagy=1;
  float x,y,h,k;
  
  if(flagx)
  {
    i+=steps;
    
    x=i/100;
    h=Ax*sin(x+phase); //_________________________
    
    k= height + 0;
//.............................
///    Serial.print("x -> ");
///    Serial.print(h);
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
    if((i>157) && (i=157))
      flagx=0;
  }

  else if(!flagx)
  {
    i-=steps;

    x=i/100;
    h=Ax*sin(x+phase); //___________________________
    
//............................
///    Serial.print("x -> ");
///    Serial.print(h);
//............................

    if((i<-157) && (i=-157)) //--------left value not decreased
      flagx=1;
  
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
///    Serial.print(" y -> ");
///    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}

//--------------------------------------------------------------------------------------------

float* find_angles (float* values)
{
  float h,k,a,b,D,perpendicular,base,alpha; //removed- ratio

  h=values[0];
  k=values[1];

  a=((h*h)+(k*k)+(f*f)-(t*t))/2;
  b=(h*h)+(k*k);    //always non-zero
  D=(4*h*h*a*a)-(4*b*((a*a)-(f*f*k*k)));

  base=((2*h*a)+sqrt(D))/(2*b);   //sign changed

  perpendicular=(-(h*base/k))+(a/k);    //there can be singularity at k=0

  if(k==0)        // condition check for k, bcoz k=0 in denominator  
  {
    k=-0.0001;
    perpendicular=(-(h*base/k))+(a/k);
    k=values[1];
  }

  alpha=(atan2((-perpendicular),(-base)))*180/pi;

  values[2]=alpha;

/*-----------alpha calculations done----------*/

  float x,y;

  x=values[0];
  y=values[1];

  float hyp,beta,cos_beta;

//  old thought process removed from here
//  only cosine rule is being applied
//  beta dependent only on x,y coordinates

  hyp=sqrt((x*x)+(y*y));
  cos_beta=((f*f)+(t*t)-(hyp*hyp))/(2*f*t);
  beta=(acos(cos_beta))*180/pi;

  values[3]=beta;

/*-----------beta calculations done-----------*/

  return values;
}
//____________________________________________________________________________

void horsewalk(float* values1, float* values2, float* values3, float* values4)
{
//#1 lb---------------------------------------------------------------------------------------------------
  legcontrol1(values1,0);
  find_angles(values1);
//................................
//  Serial.print("alpha : ");    
//  Serial.println((values1[2])); 
//  Serial.print("beta : ");     
//  Serial.println(values1[3]);   
//..............................
//  delay(5);     // delay should not be here, it should be common at the end for each iteration
//    first calculatons will be done and srvo write to be done together to avoid inter-leg lag
  
//#2 lf---------------------------------------------------------------------------------------------------
  legcontrol2(values2,0);
  find_angles(values2);  
  
//#3 rb---------------------------------------------------------------------------------------------------
  legcontrol3(values3,0);
  find_angles(values3);
//................................
//  Serial.print("alpha : ");
//  Serial.println((values3[2]));
//  Serial.print("beta : ");
//  Serial.println(values3[3]);
//................................
  
//#4 rf---------------------------------------------------------------------------------------------------
  legcontrol4(values4,0);
  find_angles(values4);

//--------------------------------------------------------------------------------------------------------
//#1 lb
  hlb_srv.write(180-(values1[2]+45+15));  //+30 is the tilt correction for hip
  klb_srv.write(180-(values1[3]+90));  //knee angle needs to be shifted by 90 always due to our setup
//#2 lf
  hlf_srv.write( (values2[2]+45+15));
  klf_srv.write( (values2[3]+90));
//#3 rb
  hrb_srv.write( (values3[2]+45+15));
  krb_srv.write( (values3[3]+90) );
//#4 rf
  hrf_srv.write(180-(values4[2]+45+15));
  krf_srv.write(180-(values4[3]+90));

  delayMicroseconds(del);
  
}


void loop() 
{
  horsewalk(values1, values2, values3, values4);
}
