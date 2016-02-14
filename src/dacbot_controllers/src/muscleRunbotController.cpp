
#include "muscleRunbotController.h"

#define STEPSIZE = 0.01
#define RADIUS = 1;

static double DEGTORAD=M_PI/180.0;

MuscleRunbotController::MuscleRunbotController(const std::string& name, const std::string& revision)
: AbstractController (name,revision){
  steps = 0;
  pos = 0;
  nSensors = 0;
  nMotors = 0;
  simulatedMass = 0.4;
  gait = new runbot::cGaitTransition(runbot::cGaitProfile());
  nnet = new runbot::cNNet((runbot::cGaitProfile*) gait);

  newGait=new runbot::cGaitProfile(78,97,115,175,2,1.5);
  nnetTwo= new runbot::cNNet((runbot::cGaitProfile*)newGait);

  gait3=new runbot::cGaitProfile(78,115,115,175,2,2.7);
  nnet3=new runbot::cNNet((runbot::cGaitProfile*)gait3);

  addParameterDef("UBC",&ubc,-0.5,-1.0,1.0,"leaning forward and backwards [1 .. -1]");
  addParameterDef("Mass",&simulatedMass,0.4,0.0,3.0,"simulated mass for muscle model");
  addInspectableValue("speed",&speed,"the speed of runbot, measured in cm/sec");
  initialized = false;
}


void MuscleRunbotController::init(int sensornumber, int motornumber, RandGen* randGen) {
nSensors = sensornumber;
nMotors =  motornumber;
steps = 0;

hipPlot.open("/home/giuliano/Documents/thesis/plots/hips.dat");//change this to your directory



motor0DerivativeVector.push_back(0);
motor0DerivativeVector.push_back(0);

leftHipDerivativeVector.push_back(0);
leftHipDerivativeVector.push_back(0);

leftDerivativeVector.push_back(0);
leftDerivativeVector.push_back(0);

rightDerivativeVector.push_back(0);
rightDerivativeVector.push_back(0);

systemFrequencyVector.push_back(0);
systemFrequencyVector.push_back(0);

freq.push_back(0);
freq.push_back(0);
shiftVector.push_back(0);
shiftVector.push_back(0);

freqDeriv.push_back(0);
freqDeriv.push_back(0);

stepFreq.push_back(0);
stepFreq.push_back(0);


frequencySystem=0;

DinLeft= new DynamicCpg(0.04);//0.04
DinRight= new DynamicCpg(0.04);//0.04


filter= new lowPass_filter(0.2);
phase=new shift_register(3);//4
rightHipDelayed= new shift_register(0);

leftKneeDelayed=new shift_register(6);
rightKneeDelayed=new shift_register(4);



/*
SigmoidTransitionFunction transF = SigmoidTransitionFunction(100);
MuscleModelConfiguration config = MuscleModelConfiguration(transF);
config.D = 1;         		//0.2
config.Dmin = 0.7;	  		//0.2
config.mass = simulatedMass;//1.5
config.radius = 0.05; 		//1
config.mActFactor = 1.4;	//1.2
config.Kmin = 0.4;	  		//0.2
config.centerAngle = M_PI;
config.groundAngle = M_PI;
config.length = 0.122;
config.timestep = 10;
LKmuscles = new DCControllingVMM(config);
addParameterDef("LKK",&(LKmuscles->K),0.8,0.0,100.0,"Stiffness of the left knee [0 .. 10.0]");
addParameterDef("LKD",&(LKmuscles->D),0.01,0.0,5,"Damping of the left knee [0 .. 10.0]");
RKmuscles = new DCControllingVMM(config);
addParameterDef("RKK",&(RKmuscles->K),0.8,0.0,100,"Stiffness of the right knee [0 .. 10.0]");
addParameterDef("RKD",&(RKmuscles->D),0.01,0.0,5,"Damping of the right knee [0 .. 10.0]");
config.centerAngle = M_PI/2.0;
config.groundAngle = M_PI/2.0;
config.length = 0.142;
config.mActFactor = 1.4; //1.5
LHmuscles = new DCControllingVMM(config);
addParameterDef("LHK",&(LHmuscles->K),0.5,0.0,100,"Stiffness of the left hip [0 .. 1.0]");
addParameterDef("LHD",&(LHmuscles->D),0.01,0.0,5,"Damping of the left hip [0 .. 10.0]");
RHmuscles = new DCControllingVMM(config);
addParameterDef("RHK",&(RHmuscles->K),0.5,0.0,100,"Stiffness of the right hip [0 .. 1.0]");
addParameterDef("RHD",&(RHmuscles->D),0.01,0.0,5,"Damping of the right hip [0 .. 10.0]");

leftMuscles = new MuscleChain(2);
leftMuscles->addMuscle(0,LKmuscles);
leftMuscles->addMuscle(1,LHmuscles);

rightMuscles = new MuscleChain(2);
rightMuscles->addMuscle(0,RKmuscles);
rightMuscles->addMuscle(1,RHmuscles);

addInspectableValue("KLHip",LHmuscles->getCurKAddress(),"current K while walking");
addInspectableValue("DLHip",LHmuscles->getCurDAddress(),"current D while walking");
*/
actualAD = valarray<double>(sensornumber+1);
initialized = true;
}

int MuscleRunbotController::getSensorNumber() const {
  return nSensors;
}

int MuscleRunbotController::getMotorNumber() const {
  return nMotors;
}




void MuscleRunbotController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {
	///IMPORTANT//
	//This function can either call the Adaptive CPG-based controller with one CPG or the CPG-based controller with
    //two CPGs. Set the variable (in muscleRunbotController.cpp) oneCPG=true for controller with one cpg
	//and oneCPG=false for controller with two CPGs, an if-else statement will switch between the two options


	if(oneCPG==true)//using only one CPG in the controller
{

	valarray<double> motorOutput(4);
	  steps++;






	  actualAD[LEFT_HIP] = sensors[0];
	  actualAD[RIGHT_HIP] = sensors[1];
	  actualAD[LEFT_KNEE] = sensors[2];
	  actualAD[RIGHT_KNEE] = sensors[3];
	  actualAD[BOOM_ANGLE] = sensors[4];
	  actualAD[LEFT_FOOT] = sensors[5];
	  actualAD[RIGHT_FOOT] = sensors[6];
	  actualAD[0] = steps;

	  double enable;
	  std::cout << steps<< std::endl;

	  double sensora;
	  double left_foot_sensor = (sensors[6]>3000)?0.0:1.0;
	  double right_foot_sensor = (sensors[5]>3000)?0.0:1.0;


      //Used to generate 1 gait with reflexive nn
	  //motorOutput=nnet->update(actualAD,steps);

	  //This is used to generate different gaits with the reflexive NN, change according to your experiment
	  if(steps < 5000)
	  	motorOutput=nnet->update(actualAD,steps);
	  else if(steps >= 5000 && steps <7000)
	    motorOutput=nnetTwo->update(actualAD,steps);
	  else  motorOutput=nnet3->update(actualAD,steps);




	  sensora=actualAD[LEFT_HIP];// feedback

	  //This is to simulate the feedback, you can cut the feedback to the CPG or you can activate it in different periods

	  // if((steps>3000 && steps<5000) || (steps >7000 && steps < 9000) || (steps >11000 && steps < 13000) || (steps >15000 && steps < 19000))
		// sensora=0;

	  // if(steps > 4000)
	    //  sensora=0;



	  std::vector<double> sign = DinLeft->generateOutputTwoLegThereshold(sensora ,motors[0],motors[2], steps );//generates motor signals
      DinLeft->setEnable(steps>2000,motors[0],  left_foot_sensor ,right_foot_sensor);// generates enable output
      enable=DinLeft->getEnable();//set enable
      std::cout << " enable: " << enable << std::endl;

      //wrtie to file, change directory
       if(steps > 500)
          {
          hipPlot << steps <<" "<<motorOutput[0] <<" "<<DinLeft->getCpg()->getFrequency()*2.1/0.015<< " "<<sign[0]<<" "<<motorOutput[0] << " " << motorOutput[2]<<" " << motorOutput[3]  <<
           	  " "<< actualAD[LEFT_HIP]<<" "<<actualAD[RIGHT_HIP]<<" " <<actualAD[LEFT_KNEE]<<" "<<
       	  	  actualAD[RIGHT_KNEE]<<" "<<actualAD[LEFT_FOOT]<< " " << actualAD[RIGHT_FOOT]<<std::endl ;
          }


      // Depending on the enable, either reflexive signals or CPG-based are sent to the motors
      motors[0] = motorOutput[0]*!enable + sign.at(0)*enable ;//LHMusclTor;
	  motors[1] = motorOutput[1]*!enable + sign.at(1)*enable ;//RHMusclTor;
	  motors[2] = motorOutput[2]*!enable + sign.at(2)*enable ;//LKMusclTor;
	  motors[3] = motorOutput[3]*!enable + sign.at(3)*enable ;//RKMusclTor;
	  motors[4] = ubc+ubc_wabl;


	    if (steps % ubc_time == 0)
	       ubc_wabl *= -1;

	     //speed measurement..
	     double stepsize = 0.01; //length of control intervalls (both should be obtained automatically somehow..)
	     double radius = 1; //armlength in global coordinates
	     if ((sensors[7]/10 - pos) < -M_PI)
	       speed = speed*0.99+0.01*((sensors[7]/10 + 2*M_PI -pos)*radius/0.01);
	     else
	       speed = speed*0.99+0.01*((sensors[7]/10 - pos)*radius)/0.01;
	     pos = sensors[7]/10;

}



	/////////////////////// two CPGs///////////////////////////



	else //using two cpgs
{

		valarray<double> motorOutput(4);
			  steps++;






			  actualAD[LEFT_HIP] = sensors[0];
			  actualAD[RIGHT_HIP] = sensors[1];
			  actualAD[LEFT_KNEE] = sensors[2];
			  actualAD[RIGHT_KNEE] = sensors[3];
			  actualAD[BOOM_ANGLE] = sensors[4];
			  actualAD[LEFT_FOOT] = sensors[5];
			  actualAD[RIGHT_FOOT] = sensors[6];
			  actualAD[0] = steps;

			  double enable;
			  std::cout << steps<< std::endl;

			  double sensora;
			  double left_foot_sensor = (sensors[6]>3000)?0.0:1.0;
			  double right_foot_sensor = (sensors[5]>3000)?0.0:1.0;



			  motorOutput=nnet->update(actualAD,steps);
			  //generating different gaits
/*
			  if(steps < 500)
			  motorOutput=nnet->update(actualAD,steps);
			  else
		      motorOutput=nnetTwo->update(actualAD,steps);
*/
			  sensora=actualAD[LEFT_HIP];

			  // Left CPG controlling left leg
			  std::vector<double> LeftLeg = DinLeft->generateOutputOneLeg(actualAD[LEFT_HIP] ,motors[0],motors[2], steps );
		      DinLeft->setEnable(steps>2000,motors[0],  left_foot_sensor ,right_foot_sensor);
		      double LeftEnable=DinLeft->getEnable();


		      // Right CPG controlling right leg
              std::vector<double> RightLeg = DinRight->generateOutputOneLeg(actualAD[RIGHT_HIP],motors[1],motors[3], steps );
              DinRight->setEnable(steps>2000,motors[1],  left_foot_sensor ,right_foot_sensor);
		      double RightEnable=DinRight->getEnable();

		      controllerEnable=LeftEnable*RightEnable;//enable set to 1 when both right and left enable are set to 1


		      std::cout << "steps " << steps << "   leftEnable" << LeftEnable <<"   righttEnable" << RightEnable << "   Enable" <<controllerEnable << std::endl;



		      if(steps > 500)
		    	  hipPlot << steps << " "<< LeftEnable<<" "<<RightEnable<< " " <<LeftLeg.at(0)<< " " <<RightLeg.at(0) <<" " << DinLeft->getCpg()->getFrequency()  << std::endl ;

		    	  //hipPlot << steps << " "<< DinLeft->getCpg()->getFrequency()*2.1/0.015<<" "<<DinRight->getCpg()->getFrequency()*2.1/0.015<< " " <<LeftLeg.at(0)<< " " <<RightLeg.at(0) <<" " << DinLeft->getCpg()->getFrequency()  << std::endl ;

		      // writing to motors either reflexive or cpg-based signals depending on enable
		      motors[0] = motorOutput[0]*!controllerEnable + LeftLeg.at(0)*controllerEnable ;//LHMusclTor;
			  motors[1] = motorOutput[1]*!controllerEnable + RightLeg.at(0)*controllerEnable ;//RHMusclTor;
			  motors[2] = motorOutput[2]*!controllerEnable + LeftLeg.at(1)*controllerEnable ;//LKMusclTor;
			  motors[3] = motorOutput[3]*!controllerEnable + RightLeg.at(1)*controllerEnable ;//RKMusclTor;
			  motors[4] = ubc+ubc_wabl;


			    if (steps % ubc_time == 0)
			       ubc_wabl *= -1;

			     //speed measurement..
			     double stepsize = 0.01; //length of control intervalls (both should be obtained automatically somehow..)
			     double radius = 1; //armlength in global coordinates
			     if ((sensors[7]/10 - pos) < -M_PI)
			       speed = speed*0.99+0.01*((sensors[7]/10 + 2*M_PI -pos)*radius/0.01);
			     else
			       speed = speed*0.99+0.01*((sensors[7]/10 - pos)*radius)/0.01;
			     pos = sensors[7]/10;

}

}



void MuscleRunbotController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

bool MuscleRunbotController::store(FILE* f) const {
}

bool MuscleRunbotController::restore(FILE* f) {
}

