#include "UserCode.hpp"
#include "UtilityFunctions.hpp"
#include "Vec3f.hpp"

#include <stdio.h> //for printf

//An example of a variable that persists beyond the function call.
float exampleVariable_float = 0.0f;  //Note the trailing 'f' in the number. This is to force single precision floating point.
Vec3f exampleVariable_Vec3f = Vec3f(0, 0, 0);
int exampleVariable_int = 0;

//We keep the last inputs and outputs around for debugging:
MainLoopInput lastMainLoopInputs;
MainLoopOutput lastMainLoopOutputs;

//Some constants that we may use:
const float mass = 32e-3f;  // mass of the quadcopter [kg]
const float gravity = 9.81f;  // acceleration of gravity [m/s^2]
const float inertia_xx = 16e-6f;  //MMOI about x axis [kg.m^2]
const float inertia_yy = inertia_xx;  //MMOI about y axis [kg.m^2]
const float inertia_zz = 29e-6f;  //MMOI about z axis [kg.m^2]
const float len = 33e-3f;
const float kap = 0.01f;

const float Jbxx = 16e-6f;
const float Jbyy = 16e-6f;
const float Jbzz = 29e-6f;


const float dt = 1.0f / 500.0f;  //[s] period between successive calls to MainLoop
Vec3f estGyroBias = Vec3f(0,0,0);
const float p = 0.001f; //Mixer for Pitch and Roll

float estRoll = 0.0f;
float estPitch = 0.0f;
float estYaw = 0.0f;

//Angles
float desRoll = 0.0f;
float desPitch = 0.0f;
float desYaw = 0.0f;

//Anglular Velocities
float desRollRate = 0.0f;
float desPitchRate = 0.0f;
float desYawRate = 0.0f;

float Cp1 = 0.0f;
float Cp2 = 0.0f;
float Cp3 = 0.0f;
float Cp4 = 0.0f;
float Csum = 0.0f;
float thrust_des = 8.0f;

Vec3f ang_acc_des = Vec3f(0.0f, 0.0f, 0.0f);
Vec3f ang_vel_des = Vec3f(0.0f, 0.0f, 0.0f);

// torques
float n1 = 0.0f;
float n2 = 0.0f;
float n3 = 0.0f;

//Height and Velocities
float estHeight = 0;
float estVelocity_1 = 0;
float estVelocity_2 = 0;
float estVelocity_3 = 0;
//Previous Height iterration
float lastHeightMeas_meas = 0;
float lastHeightMeas_time = 0;

// vertical control constants
const float natFreq_height = 0.8f;
const float dampingRatio_height = 1.5f;
float desHeight = 0.0f;

MainLoopOutput MainLoop(MainLoopInput const &in) {
  //Your code goes here!
  // The function input (named "in") is a struct of type
  // "MainLoopInput". You can understand what values it
  // contains by going to its definition (click on "MainLoopInput",
  // and then hit <F3> -- this should take you to the definition).
  // For example, "in.userInput.buttonBlue" is true if the
  // blue button is pushed, false otherwise.

  // rPY rate time constants for angle rate control
  float const timeC_rr = 0.02f;
  float const timeC_pr = timeC_rr;
  float const timeC_yr = 0.1f;

  // RPY time constants for angle control
  float const timeC_ra = 0.52f;
  float const timeC_pa = timeC_ra;
  float const timeC_ya = 0.2f;
  //Horizontal Velocity Time Constant
  const float timeC_hv = 0.9f;



  //Define the output numbers (in the struct outVals):
  MainLoopOutput outVals;
//  motorCommand1 -> located at body +x +y
//  motorCommand2 -> located at body +x -y
//  motorCommand3 -> located at body -x -y
//  motorCommand4 -> located at body -x +y

   if (in.currentTime <1.0f) {
   estGyroBias = estGyroBias + (in.imuMeasurement.rateGyro / 500.0f);
   }
   Vec3f rateGyro_corr = in.imuMeasurement.rateGyro - estGyroBias;
/*
  outVals.motorCommand1 = 0;
  outVals.motorCommand2 = 0;
  outVals.motorCommand3 = 0;
  outVals.motorCommand4 = 0;
*/

   //Angle Estimation
  if (in.currentTime > 1.0f) {
  float theta_measured = -1.0 * in.imuMeasurement.accelerometer.x / gravity;
  float phi_measured = in.imuMeasurement.accelerometer.y / gravity;
  estRoll = (1.0 - p) * (estRoll + dt * rateGyro_corr.x) + (p *phi_measured);
  estPitch = (1.0 - p) * (estPitch + dt * rateGyro_corr.y) + (p *theta_measured);
  estYaw += dt * rateGyro_corr.z;
  }


  //Predictions

  estVelocity_1 = estVelocity_1 + in.imuMeasurement.accelerometer.x*dt;
  estVelocity_2 = estVelocity_2 + in.imuMeasurement.accelerometer.y*dt;

  estHeight = estHeight + estVelocity_3*dt;
  estVelocity_3 = estVelocity_3 + 0*dt;


  //Height Estimation
  float const mixHeight = 0.3f;
  if (in.heightSensor.updated) {
    if (in.heightSensor.value < 5.0f) {
      float hMeas = in.heightSensor.value * cosf(estRoll) * cosf(estPitch);
      estHeight = (1 - mixHeight) * estHeight + mixHeight * hMeas;

      float v3Meas = (hMeas - lastHeightMeas_meas)/(in.currentTime - lastHeightMeas_time);

      estVelocity_3 = (1 - mixHeight) * estVelocity_3 + mixHeight * v3Meas;

      lastHeightMeas_meas = hMeas;
      lastHeightMeas_time = in.currentTime;
    }
  }

  //Horizontal Estimation
  float const mixHorizVel = 0.3974545f;
  float const mixmeasHorizVel = 0.86;
  if (in.opticalFlowSensor.updated) {
    float sigma_1 = in.opticalFlowSensor.value_x;
    float sigma_2 = in.opticalFlowSensor.value_y;

    float div = (cosf(estRoll) * cosf(estPitch));
    if (div > 0.5f) {
      float deltaPredict = estHeight / div;

      float v1Meas = 2 * (-sigma_1 * (mixmeasHorizVel) + in.imuMeasurement.rateGyro.y * (1 - mixmeasHorizVel)) * deltaPredict;
      float v2Meas = 2 * (-sigma_2 * (mixmeasHorizVel) - in.imuMeasurement.rateGyro.x * (1 - mixmeasHorizVel)) * deltaPredict;

      estVelocity_1 = (1 - mixHorizVel) * estVelocity_1 + mixHorizVel * v1Meas;
      estVelocity_2 = (1 - mixHorizVel) * estVelocity_2 + mixHorizVel * v2Meas;

    }
  }

  const float desAcc3 = -2 * dampingRatio_height * natFreq_height * estVelocity_3 - natFreq_height * natFreq_height * (estHeight - desHeight);
  float thrust_des = (gravity + desAcc3) / (cosf(estRoll) * cosf(estPitch));

  //Desired Horizontal Acceleration
  float desAcc1 = -(1 / timeC_hv) * estVelocity_1;
  float desAcc2 = -(1 / timeC_hv) * estVelocity_2;

  //Desired Angles
  float desRoll = -desAcc2 / gravity;
  float desPitch = desAcc1 / gravity;
  float desYaw = 0;

  outVals.telemetryOutputs_plusMinus100[0] = estRoll;
  outVals.telemetryOutputs_plusMinus100[1] = estPitch;
  outVals.telemetryOutputs_plusMinus100[2] = estYaw;

 // printf("Estimated roll angle:");
  //printf("%6.10f, ", double(estRoll));
 // printf("\n");

 // printf("Estimated pitch angle:");
 // printf("%6.10f, ", double(estPitch));
  //printf("\n");

 // printf("Estimated yaw angle:");
 // printf("%6.10f, ", double(estYaw));
 // printf("\n");


  // Desired angle velocities & angle acceleration vectors (Attitude Control)
  ang_vel_des = Vec3f( -(estRoll - desRoll) / timeC_ra, -(estPitch - desPitch) / timeC_pa, -(estYaw - desYaw) / timeC_ya );
  ang_acc_des = Vec3f( (ang_vel_des.x - rateGyro_corr.x)/timeC_rr, (ang_vel_des.y - rateGyro_corr.y)/timeC_pr, (ang_vel_des.z - rateGyro_corr.z)/timeC_yr);

  // Mixer matrix calculations
  Csum = mass*thrust_des;
  n1 = ang_acc_des.x*Jbxx;
  n2 = ang_acc_des.y*Jbyy;
  n3 = ang_acc_des.z*Jbzz;

  Cp1 = (Csum + n1/len - n2/len + n3/kap)/4.0f;
  Cp2 = (Csum - n1/len - n2/len - n3/kap)/4.0f;
  Cp3 = (Csum - n1/len + n2/len + n3/kap)/4.0f;
  Cp4 = (Csum + n1/len + n2/len - n3/kap)/4.0f;

if (in.userInput.buttonBlue) {
  //Takeoff Command
  desHeight = 1.5f;
  }
if (in.userInput.buttonGreen) {
  //Landing Command
  }

  outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(Cp1));
  outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(Cp2));
  outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(Cp3));
  outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(Cp4));
  // Computed motor force -> speeds -> PWM motor commands
  //outVals.motorCommand1 = pwmCommandFromSpeed(speedFromForce(Cp1));
  //outVals.motorCommand2 = pwmCommandFromSpeed(speedFromForce(Cp2));
  //outVals.motorCommand3 = pwmCommandFromSpeed(speedFromForce(Cp3));
  //outVals.motorCommand4 = pwmCommandFromSpeed(speedFromForce(Cp4));

  outVals.telemetryOutputs_plusMinus100[3] = estVelocity_1;
  outVals.telemetryOutputs_plusMinus100[4] = estVelocity_2;
  outVals.telemetryOutputs_plusMinus100[5] = estVelocity_3;
  outVals.telemetryOutputs_plusMinus100[6] = estHeight;
  outVals.telemetryOutputs_plusMinus100[7] = desRoll;
  outVals.telemetryOutputs_plusMinus100[8] = desPitch;

  outVals.telemetryOutputs_plusMinus100[9] = thrust_des;
  //copy the inputs and outputs:
  lastMainLoopInputs = in;
  lastMainLoopOutputs = outVals;
  return outVals;

  printf("\n");
  PrintStatus();

  return outVals;
}

void PrintStatus() {
  //For a quick reference on the printf function, see: http://www.cplusplus.com/reference/cstdio/printf/
  // Note that \n is a "new line" character.
  // Also, note that to print a `float` variable, you have to explicitly cast it to
  //  `double` in the printf function, and explicitly specify precision using something
  //  like %6.3f (six significant digits, three after the period). Example:
  //   printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //Accelerometer measurement

  printf("Last range = %6.3fm, ", \
         double(lastMainLoopInputs.heightSensor.value));
  printf("Last flow: x=%6.3f, y=%6.3f\n", \
         double(lastMainLoopInputs.opticalFlowSensor.value_x), \
         double(lastMainLoopInputs.opticalFlowSensor.value_y));

  printf("Acc:");
  printf("x=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.x));
  printf("y=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.y));
  printf("z=%6.3f, ",
         double(lastMainLoopInputs.imuMeasurement.accelerometer.z));
  //Rate gyro measurement
  printf("Gyro:");
  printf("x=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.x));
  printf("y=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.y));
  printf("z=%6.3f, ", double(lastMainLoopInputs.imuMeasurement.rateGyro.z));
  printf("\n");

  //printf("Example variable values:\n");
  //printf("  exampleVariable_int = %d\n", exampleVariable_int);
  //Note that it is somewhat annoying to print float variables.
  //  We need to cast the variable as double, and we need to specify
  //  the number of digits we want (if you used simply "%f", it would
  //  truncate to an integer.
  //  Here, we print 6 digits, with three digits after the period.
  //printf("  exampleVariable_float = %6.3f\n", double(exampleVariable_float));

  //We print the Vec3f by printing it's three components independently:
 // printf("  exampleVariable_Vec3f = (%6.3f, %6.3f, %6.3f)\n",
         //double(exampleVariable_Vec3f.x), double(exampleVariable_Vec3f.y),
         //double(exampleVariable_Vec3f.z));

  //just an example of how we would inspect the last main loop inputs and outputs:
  printf("Last main loop inputs:\n");
  printf("  batt voltage = %6.3f\n",
         double(lastMainLoopInputs.batteryVoltage.value));
  printf("  JS buttons: ");
  if (lastMainLoopInputs.userInput.buttonRed)
    printf("buttonRed ");
  if (lastMainLoopInputs.userInput.buttonGreen)
    printf("buttonGreen ");
  if (lastMainLoopInputs.userInput.buttonBlue)
    printf("buttonBlue ");
  if (lastMainLoopInputs.userInput.buttonYellow)
    printf("buttonYellow ");
  if (lastMainLoopInputs.userInput.buttonArm)
    printf("buttonArm ");
  printf("\n");
  printf("Last main loop outputs:\n");
  printf("  motor commands: = %6.3f\t%6.3f\t%6.3f\t%6.3f\t\n",
         double(lastMainLoopOutputs.motorCommand1),
         double(lastMainLoopOutputs.motorCommand2),
         double(lastMainLoopOutputs.motorCommand3),
         double(lastMainLoopOutputs.motorCommand4));
}
