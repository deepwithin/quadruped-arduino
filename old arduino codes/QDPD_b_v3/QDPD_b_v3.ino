#include <Servo.h>
#include <math.h>
// worked
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

float height = -16.5;   //height of bot from ground- a negative quantity
float values[5];
float Ax=8 ,Ay=10 , X_vel_n2p=4, X_vel_p2n=8, Y_vel=16;
/* amplitude along x
 * amplitude along y
 * velocity along X axis - negative to positive
 * velocity along Y axis - positive to negative
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

float* legcontrol(float* values, float fix, float fiy)
{
  static float i=-157, j=0, flagx=1, flagy=1;
  float x,y,h,k;
  
  if(flagx)
  {
    i+=4;
    
    x=i/100;
    h=Ax*x*100/160; //_________________________
    k= height + 0;
///    Serial.print("x -> ");
//    Serial.print("(");
///    Serial.print(h);
///    Serial.print(" y -> ");
//    Serial.print(",");
///    Serial.println(k);
//    Serial.print(") ");
    
    if(i>157)
      flagx=0;
  }

  else if(!flagx)
  {
    i-=4;

    x=i/100;
    h=Ax*x*100/160; //___________________________
///    Serial.print("x -> ");
//    Serial.print("(");
///    Serial.print(h);

    if(i<-154) //--------left value decreased
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
    k= height + (Ay*sqrt(1-(h*h/(Ax*Ax) ))); // error due to root of negative quantity
///    Serial.print(" y -> ");
//    Serial.print(",");
///    Serial.println(k);
//    Serial.print(") ");
    
  }
  values[0]=h;
  values[1]=k;
}

//--------------------------------------------------------------------

float* find_angles (float* values)
{
  float h,k,a,b,D,perpendicular,base,alpha,ratio;

  h=values[0];
  k=values[1];

  a=((h*h)+(k*k)+(f*f)-(t*t))/2;
  b=(h*h)+(k*k);    //always non-zero
  D=(4*h*h*a*a)-(4*b*((a*a)-(f*f*k*k)));

  base=((2*h*a)-sqrt(D))/(2*b);   // negative sign in front of sqroot, when changed to + gives other solution
//  Serial.println(base);
  perpendicular=(-(h*base/k))+(a/k);    //there can be singularity at k=0

  if(k==0)
  {
    k=-0.0001;
    perpendicular=(-(h*base/k))+(a/k);
    k=values[1];
  }
  //for verification
  if((int)(base*base*100)+(int)(perpendicular*perpendicular*100)==(int)(f*f))
    Serial.println("knee coordinates OK !");

//    ratio=perpendicular/base;
//    alpha=(atan(ratio))*180/pi;

  alpha=(atan2((-perpendicular),(-base)))*180/pi;

  values[2]=alpha;

/*-----------alpha calculations done----------*/

  float x,y;

  x=values[0];
  y=values[1];

  float thetaprime,hyp,theta1,sine_beta,beta,cos_beta;

//  thetaprime=atan2(abs(y),(-x));
//  theta1=thetaprime-(alpha*pi/180);// bcoz sin takes values in radians only
  hyp=sqrt((x*x)+(y*y));
//  sine_beta=hyp*sin(theta1)/t;
  cos_beta=((f*f)+(t*t)-(hyp*hyp))/(2*f*t);
//  Serial.println(sine_beta);
//  beta=(asin(sine_beta))*180/pi;
  beta=(acos(cos_beta))*180/pi;

  values[3]=beta;

/*-----------beta calculations done-----------*/

  return values;

}

void horsewalk(float* values)
{
  //#1
  legcontrol(values,pi/2,0);
  find_angles(values);
//  Serial.print("alpha : ");    //---------
//  Serial.println((values[2])); //---------
//  Serial.print("beta : ");     //---------
//  Serial.println(values[3]);   //---------
  delay(5);
  hlb_srv.write((values[2]+60));
  klb_srv.write(values[3]+90);  //knee angle needs to be shifted by 90 always
//#2
/*  legcontrol(values,pi/6,0);
  find_alpha(values);
  find_beta(values);  
  hlf_srv.write(values[2]);
  klf_srv.write(values[3]);*/
  //#3
/*  legcontrol(values,0,0);
  find_angles(values);
  Serial.print("alpha : ");
  Serial.println((values[2]));
  Serial.print("beta : ");
  Serial.println(values[3]);
  delay(100);
  hrb_srv.write(values[2]);
  krb_srv.write(values[3]);*/
  //#4
/*  legcontrol(values,pi/2,pi/2);
  find_angles(values);
  delay(50);
  hrf_srv.write(values[2]);
  krf_srv.write(values[3]);*/
  
}


void loop() 
{
  horsewalk(values);
}
