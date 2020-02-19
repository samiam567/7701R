#ifndef APID_HPP_EXISTS
#define APID_HPP_EXISTS


namespace GEAH {
  class APID {

     private:
       double one = 1; //this is the number one. (For setting the initial value of speedModifier)
       float kP, kI, kD; //the constants for the pid
       float kM;
       double pidTarget;
       double prevVelocity;
       double prevY;
       long prevTime;
       double* speedModifier;
       double porportion, integral, derivative;
       double counter;
       double* input;
       double* output;

     public:
       APID();

       void setInputPointer(double* inputValue);

       void setOutputPointer(double* outputPointer);

       double* getOutputPointer();
       void setSpeedModifier(double* newSpeedModifier);
       void setAPIDTarget(double targ);
       void setAPIDConstants(double nkP, double nkI, double nkD);
       void setKM(float kM);

       void runPid(double dt);

       void resetPID();

  };
}










#endif
