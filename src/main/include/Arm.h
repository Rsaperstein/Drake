#pragma once

// claw servo position must be public or have a public get function

#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <rev/CANSparkMax.h>
#include <ctre/Phoenix.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/buttons/POVButton.h>
#include <frc/XboxController.h>

#include <frc/Joystick.h>

#include <frc/smartdashboard/SendableChooser.h>
#include "frc/smartdashboard/SmartDashBoard.h"

using namespace rev;
using namespace std;

// in millimeters (check these)
#define armBaseHeight             342.9
#define armBaseFrontX             387.35
#define armBaseSideX              342.9
#define lowArmLength              812.8
#define highArmLength             1079.5

#define defaultX                  300 // able to change this
#define cargoHatchHeight          298
#define cargoBallHeight           1003
#define cargoBallLength           508
#define rocketHatchLowHeight      298
#define rocketBallLowHeight       749
#define rocketHatchMiddleHeight   1048
#define rocketBallMiddleHeight    1422
#define rocketHatchTopHeight      1740
#define rocketBallTopHeight       2096
#define ballPickUpX               279 
#define rocketTopHeightX          76
#define ballPickUpY               165 
#define discLoadHeight            228
#define ballLoadHeight            1101
#define ballLoadX                 368
#define rocketTopHeightBallX      76
#define startPositionX            100   // untested
#define startPositionY            300   // untested
#define yClearance                406   // only for the red bot?

#define TURRET_LEFT               998   // only for red bot
#define TURRET_RIGHT              453   // only for red bot
#define TURRET_CENTER             718.5 // only for red bot
#define TURRET_NONE               0
#define turretOffset              30.48

enum POVButtons {R, T, L, B};


using namespace frc;

class Arm {
  public:
    float turretPosition, shoulderAngle, elbowAngle, curX, curY;
    bool startPosition, startPositionReal;

    Arm(int shoulderMotor, int elbowMotor, int turretMotor, int shoulderPot);
    Arm(CANSparkMax *shoulderMotor, WPI_TalonSRX *elbowMotor, 
          WPI_TalonSRX *turretMotor, AnalogPotentiometer *shoulderPot);

    void Tick(XboxController *xbox, POVButton *dPad[4]);
    void moveToPosition(float x, float y);
    void printInfo();
  
  private:
    CANSparkMax *m_shoulderMotor;
    CANEncoder *m_shoulderMotorEncoder;
    WPI_TalonSRX *m_elbowMotor, *m_turretMotor;
    AnalogPotentiometer *m_shoulderPot;
    PIDController *m_shoulderController;

    void SetMotors();
    void ArmInit();
    bool validElbowPosition(double pos);
    double computeElbowPosition(double angle);
    double computeElbowAngle();
    bool validShoulderPosition(double pos);
    double computeShoulderPosition(double angle);
    double computeShoulderAngle();
    double computeTurretAngle();
    bool FindArmAngles(float x, float y, float *ang1, float *ang2);
    bool HardPID(CANSparkMax *motor, float currentPosition, float finalPosition, float fastThreshold, float slowThreshold);
    bool HardPID(WPI_TalonSRX *motor, float currentPosition, float finalPosition, float fastThreshold, float slowThreshold);
    // void FindArmMinMax(float base, float *elbowMin, float *elbowMax);
    float DeadZone(float input, float range);
};

