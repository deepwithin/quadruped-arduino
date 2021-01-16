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

static int j;
int iterations=(int)(628/steps);
float hlf[125];
float hlb[125];
float hrf[125];
float hrb[125];

float klf[125];
float klb[125];
float krf[125];
float krb[125];

float* legcontrol1(float* values, float phase)
{
  static float i=0, flagx=1, phaseCheck=1; // flag 1 means left to right on cartesian plane: -157 to 157
  float x,y,h,k;
  
  if(phaseCheck)
  {
    phase=100*phase;
    if(phase>=0 && phase<157)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=157 && phase<314)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=314 && phase<471)
    {
      flagx=0;
      i=i+phase;
      phaseCheck=0;
    }
    else if(phase>=471 && phase<628)
    {
      flagx=0;
      i=i+phase;
      phaseCheck=0;
    }
  }
  
  if(flagx)
  {
    i+=steps;
    
    x=i/100;
    
    h=Ax*cos(x); //_________________________
    
    k= height + 0;
//.............................
    Serial.print("x -> ");
    Serial.print(h);
    Serial.print(" y -> ");
    Serial.println(k);
//.............................
    
    if((i>=314) && (i=314))  
      flagx=0;
  }

  else if(!flagx)
  {
    i+=steps;

    x=i/100;
    h=Ax*cos(x); //___________________________
    
//............................
    Serial.print("x -> ");
    Serial.print(h);
//............................

    if((i>628) && (i=0)) //--------left value not decreased
      flagx=1;
  
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
    Serial.print(" y -> ");
    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}

float* legcontrol2(float* values, float phase)
{
  static float i=0, flagx=1, phaseCheck=1; // flag 1 means left to right on cartesian plane: -157 to 157
  float x,y,h,k;
  
  if(phaseCheck)
  {
    phase=100*phase;
    if(phase>=0 && phase<157)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=157 && phase<314)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=314 && phase<471)
    {
      flagx=0;
      i=i+phase;
      phaseCheck=0;
    }
    else if(phase>=471 && phase<628)
    {
      flagx=0;
      i=i+phase;
      phaseCheck=0;
    }
  }
  
  if(flagx)
  {
    i+=steps;
    
    x=i/100;
    
    h=Ax*sin(x); //_________________________
    
    k= height + 0;
//.............................
//    Serial.print("x -> ");
//    Serial.print(h);
//    Serial.print(" y -> ");
//    Serial.println(k);
//.............................
    
    if((i>157) && (i=157))  
      flagx=0;
  }

  else if(!flagx)
  {
    i-=steps;

    x=i/100;
    h=Ax*sin(x); //___________________________
    
//............................
//    Serial.print("x -> ");
//    Serial.print(h);
//............................

    if((i<-157) && (i=-157)) //--------left value not decreased
      flagx=1;
  
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
//    Serial.print(" y -> ");
//    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}

float* legcontrol3(float* values, float phase)
{
  static float i=0, flagx=1, phaseCheck=1; // flag 1 means left to right on cartesian plane: -157 to 157
  float x,y,h,k;
  
  if(phaseCheck)
  {
    phase=100*phase;
    if(phase>=0 && phase<157)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=157 && phase<314)
    {
      flagx=0;
      i=i+phase-157;
      phaseCheck=0;
    }
  
    else if(phase>=314 && phase<471)
    {
      flagx=0;
      i=i-phase+314;
      phaseCheck=0;
    }
    else if(phase>=471 && phase<628)
    {
      flagx=1;
      i=i-phase+314;
      phaseCheck=0;
    }
  }
  
  if(flagx)
  {
    i-=steps;
    
    x=i/100;
    
    h=Ax*sin(x); //_________________________
    
    k= height + 0;
//.............................
//    Serial.print("x -> ");
//    Serial.print(h);
//    Serial.print(" y -> ");
//    Serial.println(k);
//.............................
    
    if((i>157) && (i=157))  
      flagx=0;
  }

  else if(!flagx)
  {
    i+=steps;

    x=i/100;
    h=Ax*sin(x); //___________________________
    
//............................
//    Serial.print("x -> ");
//    Serial.print(h);
//............................

    if((i<-157) && (i=-157)) //--------left value not decreased
      flagx=1;
  
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
//.............................
//    Serial.print(" y -> ");
//    Serial.println(k);
//.............................
    
  }
  values[0]=h;
  values[1]=k;
}

float* legcontrol4(float* values, float phase)
{
  static float i=0, flagx=1, phaseCheck=1; // flag 1 means left to right on cartesian plane: -157 to 157
  float x,y,h,k;
  
  if(phaseCheck)
  {
    phase=100*phase;
    if(phase>=0 && phase<157)
    {
      flagx=1;
      i=i+phase;
      phaseCheck=0;
    }
  
    else if(phase>=157 && phase<314)
    {
      flagx=0;
      i=i+phase-157;
      phaseCheck=0;
    }
  
    else if(phase>=314 && phase<471)
    {
      flagx=0;
      i=i-phase+314;
      phaseCheck=0;
    }
    else if(phase>=471 && phase<628)
    {
      flagx=1;
      i=i-phase+314;
      phaseCheck=0;
    }
  }
  
  if(flagx)
  {
    i-=steps;
    
    x=i/100;
    
    h=Ax*sin(x); //_________________________
    
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
    i+=steps;

    x=i/100;
    h=Ax*sin(x); //___________________________
    
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

float* find_anglesMinus (float* values)
{
  float h,k,a,b,D,perpendicular,base,alpha; //removed- ratio

  h=values[0];
  k=values[1];

  a=((h*h)+(k*k)+(f*f)-(t*t))/2;
  b=(h*h)+(k*k);    //always non-zero
  D=(4*h*h*a*a)-(4*b*((a*a)-(f*f*k*k)));

  base=((2*h*a)-sqrt(D))/(2*b);   //sign changed

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

//--------------------------------------------------------------------------------------------

float* find_anglesPlus (float* values)
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
  find_anglesMinus(values1);
//................................
//  Serial.print("alpha : ");    
//  Serial.println((values1[2])); 
//  Serial.print("beta : ");     
//  Serial.println(values1[3]);   
//..............................
//  delay(5);     // delay should not be here, it should be common at the end for each iteration
//    first calculatons will be done and srvo write to be done together to avoid inter-leg lag
  
//#2 lf---------------------------------------------------------------------------------------------------
  legcontrol2(values2,3.14);
  find_anglesPlus(values2);  
  
//#3 rb---------------------------------------------------------------------------------------------------
  legcontrol3(values3,1.57);
  find_anglesPlus(values3);
//................................
//  Serial.print("alpha : ");
//  Serial.println((values3[2]));
//  Serial.print("beta : ");
//  Serial.println(values3[3]);
//................................
  
//#4 rf---------------------------------------------------------------------------------------------------
  legcontrol4(values4,4.71);
  find_anglesMinus(values4);

//--------------------------------------------------------------------------------------------------------
//#1 lb
  hlb[j]= (values1[2]+30);  //+30 is the tilt correction for hip
  klb[j]= (values1[3]+60);  //knee angle needs to be shifted by 90 always due to our setup
//#2 lf
  hlf[j]= (values2[2]+30);
  klf[j]= (values2[3]+60);
//#3 rb
  hrb[j]= (values3[2]+30);
  krb[j]= (values3[3]+60);
//#4 rf
  hrf[j]= (values4[2]+30);
  krf[j]= (values4[3]+60);
  
}

void setup()
{
  hlf_srv.attach(5);
  hlb_srv.attach(7);
  hrf_srv.attach(11);
  hrb_srv.attach(10);
  
  klf_srv.attach(4);
  klb_srv.attach(6);
  krf_srv.attach(12);
  krb_srv.attach(9);

  Serial.begin(9600);

  for(j=0; j < iterations; j++)
  {
    horsewalk(values1, values2, values3, values4);
  }
  
}

void loop() 
{
  for(int n=0; n < iterations; n++)
  {
    //#1 lb
    hlb_srv.write(hlb[t]);  
    klb_srv.write(klb[t]);  
    //#2 lf
    hlf_srv.write(hlf[t]);
    klf_srv.write(klf[t]);
    //#3 rb
    hrb_srv.write(hrb[t]);
    krb_srv.write(krb[t]);
    //#4 rf
    hrf_srv.write(hrf[t]);
    krf_srv.write(krf[t]);

    delayMicroseconds(del);
  }
  delayMicroseconds(del);
}
