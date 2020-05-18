// VEX V5 C++ Project with Competition Template
#include "vex.h"
using namespace vex;
#define DELTA 800;
#define SSUM 20;
//I wasn't sure what unit to use fro DELTA, so I stuck to 800, Ashwin's original value for DELTA


//#region config_globals
vex::brain      Brain;
vex::motor      LFC(vex::PORT1, vex::gearSetting::ratio18_1, false);
vex::motor      RFC(vex::PORT2, vex::gearSetting::ratio18_1, false);
vex::motor      LBC(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor      RBC(vex::PORT4, vex::gearSetting::ratio18_1, false);
vex::motor      Rl(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::motor      Intake(vex::PORT6, vex::gearSetting::ratio18_1, false);
vex::motor      ll(vex::PORT7, vex::gearSetting::ratio18_1, false);
vex::controller controller(vex::controllerType::primary);
vex::pot        intk_pot(Brain.ThreeWirePort.B);
//#endregion config_globals
// :)


class closedLoop {
 public:
 float kI = 0;
 float kD= 0;
 float kP =0;
 float target;
 double error =0; 
 int minPower =0;
 int minDrivingPower = 1100;
 int maxPower = 11500;
 double pPwr;
 double motorSpeed;
 int input;
int power;
int lastError;
double iPwr;
double dPwr;
int limit = 300;
int integral = 0;
int derivative =0; 

int pwr(target, input)
    
    void PIDPower(int p, int i, int d){
    p = kP;
    i = kI;
    d = kD;
    
    }
}

double closedLoop::pPwr{
    pPwr=kP*error
    error= target-input; 
    
    power = pPwr + iPwr + dPwr;
    
      if (abs(error) < limit){
            
            {
                integral += error;
            }
            
            else
            {
                integral = 0;
                }
            }
      }
    
     derivative = error - lastError; 
        lastError = error;
        
    if(double power > maxPower){
        power= 12000;
    }
    if (double power < minDrivingPower){
        power = 1100;
    }
    return power;
    sleepMs (15);
}
closedLoop pid;

class chassis{
    public: 
    float rpwr;
    float lpwr;
    int ctarget = 0;
    void chassisAbsolute(int c_target, int rvelo, int c_MS);

}

void chassis::chassisAbsolute(int c_target, int rvelo, int c_MS){
c_target = ctarget;
rvelo = pid.maxPower
        vex::sleepMs(waitMS);
}
class lift{
    public:
    void toggle();
    void preset();
    int liftpwr;
    int ltarget; 
    bool LPID;
    int ltarget;
    void toggle;
    void linit;
    int output;
    int input;
    void threetoggleheight;
    void intake
    int spot;
}

void lift::linit(){
    output=0;
    input=2550;
    ltarget=0;
    lpressedno = 1;
}

void lift::toggle(){
      if(e_controller.ButtonR1.pressing() && !R1)
    {
      lpressedno ++;
      (lpressednot%2 == 0) ? ltarget = 160 : ltarget = -15;
}

void lift::threetoggleheight(){
     if(e_controller.ButtonA.pressing() && !A)
    {
      ltarget = 230; //low tower height
    }
    if(e_controller.ButtonX.pressing() && !X)
    {
      ltarget = 260; //mid tower height
    }
    if(e_controller.ButtonB.pressing() && !B)
    {
      ltarget = 300; //top tower height
    }
}

void lift::intake(){

      if(e_controller.ButtonR2.pressing() && !R2)
      {
        output++;
        if(output%2 == 0)
        {
          input = 2550;
        }
        if(output%2 != 0)
        {
          input = 300;
        }
}
int numerator = 1;
chassis Work;
int variable;
bool chassisTurn = false;

void chassisturnthread(){
    While(chassisTurn){
    Work.rpwr = pid.pwr(Work.c_target,RBC.rotation(vex::rotationUnits deg));
    Work.lpwr = pid.pwr(Work.c_target, LBC.rotation(vex::rotationUnits deg));
    int currentBatteryMV= int(RBC.voltage(vex::voltageUnits::mV));
    int PWR = abs(int( Work.rpwr));
    int batteryVoltage = currentBatteryMV- PWR // batteryVoltage is the sum of the current battery minus the power that PWR will take away from the battery voltage.
    if(batteryVoltage > DELTA){
        Work.rpwr += int (Work.rpwr)* SSUM; //if batteryVoltage is lower than DELTA, rpwr and lpwr will increase by SSUM, and previous rpwr and lpwr values are multiplied by SSUM
        Work.lpwr+ = int (Work.lpwr)* SSUM;//which in turn, batteryVoltage slowly decreases and the extra power given to rpwr and lpwr ends when it reaches 800
    }
    if(abs (int(Work.rpwr)>variable)){
        Work.rpwr += int(Work.rpwr)* variable;
        Work,lpwr += int(Work.lpwr)* variable;
    }
    RFC.spin(vex::directionType fwd, Work.rpwr* numerator, vex::voltageUnits::mV);
    LFC.spin(vex::directionType fwd, Work.lpwr* numerator, vex::voltageUnits::mV);
    RBC.spin(vex::directionType fwd, Work.rpwr, vex::voltageUnits::mV);
    LBC.spin(vex::directionType fwd, Work.rpwr, vex::voltageUnits::mV);
    sleepMs(15);
}
}

void chassisthread(){
    While(!chassisTurn){
      Work.rpwr = pid.pwr(Work.c_target,RBC.rotation(vex::rotationUnits deg));
    Work.lpwr = pid.pwr(-Work.c_target, LBC.rotation(vex::rotationUnits deg));
    int currentBatteryMV= int(RBC.voltage(vex::voltageUnits::mV));
    int PWR = abs(int( Work.rpwr));
    int batteryVoltage = currentBatteryMV- PWR // batteryVoltage is the sum of the current battery minus the power that PWR will take away from the battery voltage.
    if(batteryVoltage > DELTA){
        Work.rpwr += int (Work.rpwr)* SSUM; //if batteryVoltage is lower than DELTA, rpwr and lpwr will increase by SSUM, and previous rpwr and lpwr values are multiplied by SSUM
        Work.lpwr+ = int (Work.lpwr)* SSUM;//which in turn, batteryVoltage slowly decreases and the extra power given to rpwr and lpwr ends when it reaches 800
    }
    if(abs (int(Work.rpwr)>variable)){
        Work.rpwr += int(Work.rpwr)* variable;
        Work,lpwr += int(Work.lpwr)* variable;
    }
    RFC.spin(vex::directionType fwd, Work.rpwr* numerator, vex::voltageUnits::mV);
    LFC.spin(vex::directionType fwd, Work.lpwr* numerator, vex::voltageUnits::mV);
    RBC.spin(vex::directionType fwd, Work.rpwr, vex::voltageUnits::mV);
    LBC.spin(vex::directionType fwd, Work.rpwr, vex::voltageUnits::mV);
    sleepMs(15);
}
}

lift Lift;
int potValue;
int int_speed;
void intakethread(){
    while(1){
    potValue = intk_pot.value(analogUnits::range12bit);
    int_speed= pid.pwr(Lift.spot ,potValue);
    Intake.spin(directionType fwd, int_speed, vex::voltageUnits::mV );
    sleepMs (15);
}
}
int mVlift = 10000;
void liftpowerthread(){
    while (1){
        liftpwr = pid.pwr(Lift.ltarget, ll.rotation(vex::rotationUnits deg));
        (liftpwr>mVlift)? liftpwr= mVlift; //lift is easy to burn out, so limits are set
        (liftpwr<-mVlift)? liftpwr= -mVlift;
        ll.spin(directionType:: fwd, liftpwr, vex::voltageUnits:: mV);
        Rl.spin(directionType::fwd, liftpwr, vex::voltageUnits:: mV);
    }
}





// Creates a competition object that allows access to Competition methods.
vex::competition Competition;

void pre_auton() {
    vex::thread ct(chassisthread);
    vex::thread ctt(chassisturnthread);
    vex::thread it(intakethread);
    vex::thread lt(liftpowerthread);
    pid.PIDPower(0.1,0,0);
     Lift.input = 2550;
    RBC.resetRotation();
   
}

void autonomous() {
  
 Lift.PIDPower(110,0,0);
    chassisturnthread.PIDPower(40,10,10);
    chassisthread.PIDPower(40,0,10);
 chassisthread.chassisAbsolute(-200, 0, 500);
	Lift.ltarget = 160;
	variable = 3500;

	Lift.PIDPower(150, 0, 0);
	chassisthread.chassisAbsolute(-560, 0, 1600);
	Lift.ltarget = -10; 
	sleepMs(500); 


	Lift.ltarget = 250; 
	sleepMs(550);
	chassisthread.chassisAbsolute(-710, 0, 1000);


	Lift.ltarget = -10;
	sleepMs(500);


	variable = 2500;
	Lift.ltarget = 250;
	sleepMs(550);
	chassisthread.chassisAbsolute(-875, 0, 1100);


	Lift.ltarget = -20;
	sleepMs(500);


	Lift.ltarget = 100;
	sleepMs(300);
	chassisthread.chassisAbsolute(-220, 0, 1000);

	variable = 5000;
	chassisthread.ctarget = -400;
	sleepMs(2000);

	ct.interrupt();
	ctt.interrupt();

	RBC.resetRotation();
	LBC.resetRotation();
	sleepMs(25);
	chassisTurn = true;
	vex::thread _ctt(chassisturnthread);
	chassisturnthread.ctarget = 335;
	sleepMs(1000);
	_ctt.interrupt();

	RBC.resetRotation();
	LBC.resetRotation();
	sleepMs(10);
	chassisTurn = false;
	vex::thread ctt(chassisturnthread);

	pid.PIDPower(0.1, 0, 0);

	chassisthread.ctarget = -700; 
	sleepMs(1250);
	Lift.ltarget = 0;
	sleepMs(300);
	Lift.input = 500;
	sleepMs(300);
	mvLift = 6500;

	numerator = 0;
	Lift.ltarget = 300;
	sleepMs(1400);
	variable = 10000;
	chassisthread.ctarget = -200;

	sleepMs(500);   

}
void drivercontrol() {
    // Place drive control code here, inside the loop
    while (true) {
        LFC.spin(directionType::fwd, e_controller.Axis3.value(vex::percentUnits::pct),vex::velocityUnits::mV);
        LBC.spin(directionType::fwd, e_controller.Axis3.value(vex::percentUnits::pct),vex::velocityUnits::mV);
        RFC.spin(directionType::fwd, e_controller.Axis2.value(vex::percentUnits::pct),vex::velocityUnits::mV);
        RBC.spin(directionType::fwd, e_controller.Axis2.value(vex::percentUnits::pct),vex::velocityUnits::mV);
        
    }
}

int main() {
    // Do not adjust the lines below

    // Set up (but don't start) callbacks for autonomous and driver control periods.
    Competition.autonomous(autonomous);
    Competition.drivercontrol(drivercontrol);

    // Run the pre-autonomous function.
    pre_auton();

    // Robot Mesh Studio runtime continues to run until all threads and
    // competition callbacks are finished.
}