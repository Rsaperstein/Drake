/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


// check comments in arm cpp lines 1&2
// check comments in dalekdrive cpp line 280

#include <iostream>
#include "Drake.h"

void
Robot::RobotInit() 
{   
    m_drive      = new DalekDrive(1, 2, 3, 4, DalekDrive::driveType::kMecanum);
    m_leftStick  = new frc::Joystick(1);
    //m_rightStick = new frc::Joystick(2);
    m_xbox       = new frc::XboxController(3);
    m_dPad[R]    = new frc::POVButton(*m_xbox, 90);
    m_dPad[T]    = new frc::POVButton(*m_xbox, 0);
    m_dPad[L]    = new frc::POVButton(*m_xbox, 270);
    m_dPad[B]    = new frc::POVButton(*m_xbox, 180);

    m_arm = new Arm(SHOULDER_MOTOR, ELBOW_MOTOR, TURRET_MOTOR, 0);
    m_claw = new Claw(CLAW_MOTOR, 0);
    // CameraServer::GetInstance()->StartAutomaticCapture();

#ifdef USE_LIDAR
    microLidar = new MicroLidar("/dev/i2c-2", MicroLidar::CONTINUOUS_MEASURE_MODE);
        for (int i = 0; i < LIDAR_COUNT; i++) {
            microLidar->Add(i);
        }
        microLidar->InitSensors();
        microLidar->StartMeasurements();
#endif
    lineSensor = new LineSensor();
    dalekShuffleboard = new DalekShuffleboard(microLidar, lineSensor);
    ahrs = new AHRS(SPI::Port::kMXP);
    
    // CameraServer::GetInstance()->StartAutomaticCapture();
}

void
Robot::RobotPeriodic() 
{
#ifdef USE_LIDAR
    microLidar->PollDevices();
#endif
    dalekShuffleboard->continious();
}

void
Robot::AutonomousInit() 
{
    TeleopInit();
}

void
Robot::AutonomousPeriodic()
{
    TeleopPeriodic();
}

void
Robot::TeleopInit() 
{
    // How were are facing in now 0
    ahrs->ZeroYaw();
}

void
Robot::TeleopPeriodic()
{
    SmartDashboard::PutBoolean("Dpad[L]", m_dPad[L]->Get());

    bool calibrated = !(ahrs->IsCalibrating());
    SmartDashboard::PutBoolean("NAV-X calibrated", calibrated);

    // pick one to test, all should in principle work for the mecanum wheels
    // m_drive->TankDrive(m_leftStick, m_rightStick, false);
    // m_drive->Polar(m_leftStick, m_rightStick);
    // m_drive->Cartesian(m_leftStick, m_rightStick, 0.0);
    // m_drive->SetLeftRightMotorOutputs(m_leftStick->GetY(), -m_rightStick->GetY());
    if(calibrated) {
        SmartDashboard::PutNumber("Robot Heading", ahrs->GetFusedHeading());
        m_drive->Cartesian(m_leftStick, 0.0);
        m_claw->Tick(m_xbox);
        m_arm->Tick(m_xbox, m_dPad);
    }    
    // print subsystems information
    m_arm->printInfo();
    m_claw->printVoltage();

    /*bool line = lineSensor->getLineSensor(1);
        
        if(m_leftStick->GetTrigger()){
            if(!line){
                m_drive->SetLeftRightMotorOutputs(0.05, -0.05);
            }
            else{
                m_drive->SetLeftRightMotorOutputs(0.0, 0.0);
            }
        }
        else{
            m_drive->SetLeftRightMotorOutputs(0.0, 0.0);
        }*/
}

void
Robot::TestPeriodic()
{

}

#ifndef RUNNING_FRC_TESTS
int
main() 
{ 
  return frc::StartRobot<Robot>();
}
#endif
