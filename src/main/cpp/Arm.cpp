// for now, both motors will move into position at the same time... 
// however this might need to change in order to keep in bounds and 
// not hit walls in front of us 
#include "Drake.h"

Arm::Arm(int shoulderMotor, int elbowMotor, int turretMotor, int shoulderPot)
{
    m_shoulderMotor        = new CANSparkMax(shoulderMotor, CANSparkMax::MotorType::kBrushless);
    m_elbowMotor           = new WPI_TalonSRX(elbowMotor);
    m_turretMotor          = new WPI_TalonSRX(turretMotor);
    m_shoulderPot          = new AnalogPotentiometer(shoulderPot, 1.0, 0.0);
    m_shoulderController   = new PIDController(3.2, 0.0, 0.0, m_shoulderPot, m_shoulderMotor);
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    ArmInit();
}

Arm::Arm(CANSparkMax *shoulderMotor, WPI_TalonSRX *elbowMotor, WPI_TalonSRX *turretMotor, AnalogPotentiometer *shoulderPot)
{
    m_shoulderMotor        = shoulderMotor;
    m_elbowMotor           = elbowMotor;
    m_turretMotor          = turretMotor;
    m_shoulderPot          = shoulderPot;
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    m_shoulderController   = new PIDController(3.2, 0.0, 0.0, m_shoulderPot, m_shoulderMotor);
    ArmInit();
}

void
Arm::ArmInit()
{
    // needed to configure the talon to make sure things are ready for position mode
    m_elbowMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
    m_elbowMotor->SetSensorPhase(false); // Need to test this value for black bot
    m_elbowMotor->ConfigFeedbackNotContinuous(true);
    m_elbowMotor->ConfigAllowableClosedloopError(0, 0, 0);
    m_elbowMotor->Config_IntegralZone(0, 20.0, 0);
    m_elbowMotor->Config_kF(0, 0.0, 0);
	m_elbowMotor->Config_kP(0, 7.2, 0);
	m_elbowMotor->Config_kI(0, 0.0, 0);
    m_elbowMotor->Config_kD(0, 2.0, 0);

    // shoulder motor PID control
    m_shoulderController->SetContinuous(false);
    m_shoulderController->Reset();

    fetalPosition = true; // once we find where this is we need to set curX and curY and moveToPosition accordingly
}

float
Arm::DeadZone(float input, float range) 
{
    if (abs(input) < range) {
        return 0;
    } else {
        if (input > 0) {
            return (input - range) / (1 - range);
        } else {
            return -1 * (abs(input) - range) / (1 - range);
        }
    }
}

void
Arm::Tick(XboxController *xbox, POVButton *dPad[])
{
    float turretMove = DeadZone(xbox->GetX(GenericHID::JoystickHand::kRightHand), .3) * .5;
    float x = curX;
    float y = curY;
    bool move = true;
    if (xbox->GetAButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchLowHeight;
        } else if (dPad[T]->Get()) {
            x = cargoBallLength;
            y = cargoBallHeight;
        } else if (dPad[L]->Get()) {
            x = defaultX;
            y = cargoHatchHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallLowHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetBButton()) {
        if (dPad[R]->Get() || dPad[T]->Get() || dPad[L]->Get() || dPad[B]->Get()) {
            x = defaultX;
            y = ballLoadHeight;
        } else {
            x = ballPickUpX;
            y = ballPickUpY;
        }
    } else if (xbox->GetXButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchMiddleHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallMiddleHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetYButton()) {
        if (dPad[R]->Get()) {
            x = defaultX;
            y = rocketHatchTopHeight;
        } else if (dPad[B]->Get()) {
            x = defaultX;
            y = rocketBallTopHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetTriggerAxis(GenericHID::JoystickHand::kRightHand) > .1) {
        // x =
        // y =
        fetalPosition = true;
        turretPosition = TURRET_CENTER;
    } else {
        move = false;
        m_elbowMotor->Set(DeadZone(xbox->GetY(GenericHID::JoystickHand::kRightHand), .4) * .6); // maybe change 6
        m_shoulderMotor->Set(DeadZone(xbox->GetY(GenericHID::JoystickHand::kLeftHand), .4) * -.2);  // same as above
        std::cout << "X: " << x << "\n" << "Y: " << y << "\n";
    }
    if (move) {
        moveToPosition(x, y);
    } else {
        if (xbox->GetBackButton()) {
            turretPosition = TURRET_LEFT;
        } else if (xbox->GetBackButton()) {
            turretPosition = TURRET_RIGHT;
        } else if (xbox->GetStickButton(GenericHID::JoystickHand::kRightHand)) {
            turretPosition = TURRET_CENTER;
        } else {
            if (turretMove != 0) {
                turretPosition = TURRET_NONE;
                m_turretMotor->Set(turretMove);
            }
        }
    }
    SetMotors();
}

void
Arm::moveToPosition(float x, float y)
{
    float ang1 = -1, ang2 = -1;
    if (FindArmAngles(x, y, &ang1, &ang2)) {
        shoulderAngle = ang1;
        elbowAngle = ang2;
        curX = x;
        curY = y;
        std::cout << "MOVING TO: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    } else {
        std::cout << "INVALID ANGLE: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    }
}

double
Arm::computeElbowPosition(double angle)
{
#ifdef RED_BOT
    return -169.284 * angle + 655.854;
#else
    // need BLACK_BOT numbers...for now if defined using red
    return -169.284 * angle + 655.854;
#endif
}

bool
Arm::validElbowPosition(double pos)
{
#ifdef RED_BOT
    if((pos < 200.0) || (pos > 600.0)) {
        std::cout << "Elbow position out of range: " << pos << "\n";
        return false;
    }
    return true;
#else
    //TBD: need black numbers
    return true;
#endif
}

double
Arm::computeShoulderPosition(double angle)
{
#ifdef RED_BOT
    return angle * 0.0837496 + 0.393355;
#else // BLACK_BOT
    return angle * 0.163044 + 0.142698;
#endif
}

bool
Arm::validShoulderPosition(double pos)
{
    // need to figure out valid values for red and black bots
    return true;
}

// the functions used to set the setpoint needed to change arm positions
// to currently requested angles
void
Arm::SetMotors() 
{
    double elbowPosition;
    double shoulderPosition;

    // Compute the position that the elbow and shoulder potentiometers should
    // be at and tell the motorcontroller to go there via the PID closed loop.
    // The talon can do this itself, for the shoulder motor controller we need
    // to do the PID control loop in software.  Elbow has a fairly large error
    // which varies over the range +/- 20 units.  Shoulder moves slowly to it's
    // position, which may or may not be an issue.

    if (fetalPosition) {
        // turret PID to center
        //if turret is at ~center
            // enable elbow and shoulder movement
    } else {
        elbowPosition = computeElbowPosition(elbowAngle);
        if(validElbowPosition(elbowPosition)) {
            m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, elbowPosition);
        }
    
        shoulderPosition = computeShoulderPosition(shoulderAngle);
        if(validShoulderPosition(shoulderPosition)) {
            m_shoulderController->SetSetpoint(shoulderPosition);
            m_shoulderController->SetEnabled(true);
        }

        if (turretPosition != TURRET_NONE) {
            m_turretMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, turretPosition);
        }
    }
}

// this function takes in the x distance from the target 
// starting from the edge of the drive train, and the y
// from the ground, and computes the required arm angles.
bool
Arm::FindArmAngles(float x, float y, float *ang1, float *ang2)
{
    float r;

    //TBD: must make the x value vary based on where the turret is.
    // for now i assume it is facing forward
	y -= armBaseHeight;

    x += armBaseFrontX;
    r = sqrt(x*x+y*y);
	*ang2 = acos((highArmLength * highArmLength + lowArmLength * lowArmLength - r * r) / (2 * highArmLength * lowArmLength));
	*ang1 = acos((lowArmLength * lowArmLength + r * r - highArmLength * highArmLength) / (2 * lowArmLength * r)) + atan(y / x);
    // HERE must find out whether or not the angles are allowed on the robot and return true or false accordingly
    return (*ang1 > 0 && *ang2 > 0);
}

void 
Arm::printInfo()
{
    //min: .156 max: .158
    SmartDashboard::PutNumber("Shoulder current", m_shoulderMotor->GetOutputCurrent());
    //min:.125 max:.375
    SmartDashboard::PutNumber("Elbow current", m_elbowMotor->GetOutputCurrent());
    //min:.125 max:.8
    SmartDashboard::PutNumber("Turret current", m_turretMotor->GetOutputCurrent());

    SmartDashboard::PutNumber("Elbow position", m_elbowMotor->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Elbow close loop error", m_elbowMotor->GetClosedLoopError(0));

    SmartDashboard::PutNumber("Shoulder position", m_shoulderPot->Get());
    SmartDashboard::PutNumber("Shoulder close loop error", m_shoulderController->GetError());
}



// hard coded PID save 

/*if (abs(m_shoulderPot->Get() - (shoulderAngle * 0.163044 + 0.142698)) > .01) { // .1 is a placeholder for how close the motor can get at full power
        if (m_shoulderPot->Get() > (shoulderAngle * 0.163044 + 0.142698)) {
            m_shoulderMotor->Set(-1); // too fast?
        } else {
            m_shoulderMotor->Set(1); // same
        }
    } else {
        if (abs(m_shoulderPot->Get() - (shoulderAngle * 0.163044 + 0.142698)) > .001) { // .01 is a placeholder for how close to let the motor get without any extra adjustment
            if (m_shoulderPot->Get() > (shoulderAngle * 0.163044 + 0.142698)) {
                m_shoulderMotor->Set(-.1); // this is a guess
            } else {
                m_shoulderMotor->Set(.1); // also a guess
            }
        } else {
            m_shoulderMotor->Set(0);
        }
    }*/