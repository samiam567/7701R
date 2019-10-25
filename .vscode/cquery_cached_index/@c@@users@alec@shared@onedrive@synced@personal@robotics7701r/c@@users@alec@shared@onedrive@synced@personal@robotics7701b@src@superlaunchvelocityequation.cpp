//OUTDATED 2018

/*
#include "generalFunctions.h"
#include "Settings.h"

 double Lp = .051;
 double Lpc = .135;
 double r = 0.072;
 double Y2 = .3;
 double Y4 = .19;
 double z = .075;
 double Px = -0.075;
 double Lc = .22; //physical length of cannon
 double Hc = 0.2; //physical height of the axle that aims the cannon


 //settings
  double deltaX = Lp/1000;
  double preassure = 115; //psi
  double Mball = .056;
  double coefficientOfFriction = 0.075;
  double numberOfRubberBands = 2; //number of doubled up rubber bands

  //rubber band
  double Dseq = .0425;
  double Krb = 150;
  double Rs = .06;


  //piston
 double plungerRadiusInches = .125;
 double Fpiston;

 double Gm = 980/100;
 double ThetaPis,ThetaBar,ThetaCan;
 double q,Tbar,x,Dball,FbarIball,FfrictionIball;
 double Trb;


 double calculateThetaPis(double x) { //y5
  double Beta = sqrt( Px*Px + Y4*Y4 );

  ThetaPis = atan(Y4/(-Px)) - acos( ( pow(Lpc+x,2) + Beta*Beta - r*r ) / (2 * (Lpc+x) * Beta));
  return ThetaPis;
}

 double calculateThetaBar(double x, double ThetaPis) { //y6
  ThetaBar = atan( Px/Y4 + 1/tan(ThetaPis) );
  return ThetaBar;
}

 double calculateQ(double ThetaCan, double ThetaBar) { //y7
  q = ( (z*tan(ThetaCan) + Y2) / (tan(ThetaCan) + 1/tan(ThetaBar)) ) / sin(ThetaBar);
  return q;
}



 double calculateTBar(double ThetaPis, double ThetaBar) { //y8
  Tbar = r*Fpiston*cos(ThetaPis-ThetaBar);
  return Tbar;
}

 double calculateFbarIball(double Tbar, double ThetaBar, double ThetaCan) { //y9
  FbarIball = (Tbar/q) * cos(ThetaBar-ThetaCan);
  return FbarIball;
}

 double calculateDBall(double ThetaCan, double ThetaBar) {  //y0
  Dball = ( ( (z*tan(ThetaCan) + Y2) / (tan(ThetaCan) + 1/tan(ThetaBar)) ) - z) / cos(ThetaCan);
  return Dball;
}

  double calculateFfrictionIball(double Tbar, double ThetaBar, double ThetaCan, double q) {
    double FnormalcannonIball = Tbar/q * sin(ThetaBar-ThetaCan) - Mball*Gm;
    FfrictionIball = fabs(coefficientOfFriction*FnormalcannonIball);
    return FfrictionIball;
  }

 double calculateTrb(double ThetaBar) {
      Trb = numberOfRubberBands*Rs*Krb*(Rs*(90-ThetaBar));
      return Trb;
  }
 double calculateSuperLaunchVelocity(double cannonAngle) {
  try{
    ThetaCan = PI * cannonAngle/180;

    Fpiston = preassure * (plungerRadiusInches*plungerRadiusInches)*PI * 4.448222; //25N


    double MomentOfForce = 0;
    for (double x = 0; x < (Lp); x+=deltaX) {
       calculateThetaPis(x);
       calculateThetaBar(x,ThetaPis);
       calculateQ(ThetaCan,ThetaBar);

       calculateTBar(ThetaPis,ThetaBar);
       calculateFbarIball(Tbar, ThetaBar,ThetaCan);
       MomentOfForce += (FbarIball - FfrictionIball)*deltaX;
    }


    double WIball = (MomentOfForce / Lp) * .21;///( calculateDBall(calculateThetaPis(Lp),calculateThetaBar(Lp,calculateThetaPis(Lp))) - calculateDBall(calculateThetaPis(0),calculateThetaBar(0,calculateThetaPis(0))) );
    WIball -= Mball * Gm * Lc * sin(ThetaCan);
    double Vball = sqrt(2*WIball/Mball);

    return Vball;
  }catch(int e) {
    return ERROR;
  }
}

*/
