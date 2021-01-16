float* trajectory(float* values, float phase)
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
  
    elseif(phase>=157 && phase<314)
    {
      flagx=0;
      i=i+phase-157;
      phaseCheck=0;
    }
  
    elseif(phase>=314 && phase<471)
    {
      flagx=0;
      i=i-phase+314;
      phaseCheck=0;
    }
    elseif(phase>=471 && phase<628)
    {
      flagx=1;
      i=i-phase+314;
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
