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
    m_shoulderController   = new PIDController(0.0, 0.0, 0.0, m_shoulderPot, m_shoulderMotor);
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    ArmInit();
}

Arm::Arm(CANSparkMax *shoulderMotor, WPI_TalonSRX *elbowMotor, WPI_TalonSRX *turretMotor, AnalogPotentiometer *shoulderPot, MicroLidar *microLidar)
{
    m_shoulderMotor        = shoulderMotor;
    m_elbowMotor           = elbowMotor;
    m_turretMotor          = turretMotor;
    m_shoulderPot          = shoulderPot;
    m_shoulderMotorEncoder = new CANEncoder(*m_shoulderMotor);
    m_shoulderController   = new PIDController(0.0, 0.0, 0.0, m_shoulderPot, m_shoulderMotor);
    #ifdef USE_LIDAR
        this->microLidar = microLidar;
    #endif
    ArmInit();
}

void
Arm::ArmInit()
{
    // needed to configure the talon to make sure things are ready for position mode
    m_elbowMotor->SetNeutralMode(NeutralMode::Brake);
    m_elbowMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
    m_elbowMotor->SetSensorPhase(false); // Need to test this value for black bot
    m_elbowMotor->ConfigFeedbackNotContinuous(true);
    m_elbowMotor->ConfigAllowableClosedloopError(0, 0, 0);
    m_elbowMotor->Config_IntegralZone(0, 0, 0);
    m_elbowMotor->Config_kF(0, 0.0, 0);
	m_elbowMotor->Config_kP(0, 5.0, 0);
	m_elbowMotor->Config_kI(0, 0.001, 0);
    m_elbowMotor->Config_kD(0, 4.0, 0);
    m_elbowMotor->ConfigClosedloopRamp(0.75);
    m_elbowMotor->ConfigPeakCurrentDuration(100);
    m_elbowMotor->ConfigPeakCurrentLimit(40);

    // shoulder motor PID control
    //m_shoulderController->SetPID(7.0, 0.0, 0.0);
    //m_shoulderController->SetContinuous(false);
    //m_shoulderController->Reset();

 	m_shoulderMotor->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_shoulderMotor->SetSmartCurrentLimit(40, 2, 0);
	m_shoulderMotor->SetOpenLoopRampRate(0.5);

    m_turretMotor->SetNeutralMode(NeutralMode::Brake);
    m_turretMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::Analog, 0, 0);
    m_turretMotor->ConfigFeedbackNotContinuous(true);
    m_turretMotor->ConfigAllowableClosedloopError(0, 0, 0);
    m_turretMotor->ConfigPeakCurrentDuration(100);
    m_turretMotor->ConfigPeakCurrentLimit(40);
    m_turretMotor->ConfigClosedloopRamp(0.5);

    shoulderOverride = false;
    init = true;

    //Starting position
    curX = 100; //609.6
    curY = 300; //609.6
    moveToPosition(curX, curY);
    startPosition = false;
    startPositionReal = false;
    turretPosition = TURRET_NONE;
    // eventually replace this by somehow going to the starting position
}

float
Arm::DeadZone(float input, float range) 
{
    if (abs(input) < range) {
        return 0;
    } else {
        if (input > 0) {
            return (abs(input) - range) / (1 - range);
        } else {
            return -1 * (abs(input) - range) / (1 - range);
        }
    }
}

void
Arm::Tick(XboxController *xbox, POVButton *dPad[])
{   
    float turretMove = DeadZone(xbox->GetX(GenericHID::JoystickHand::kRightHand), .3) * .5;
    float x = curX, y = curY;
    bool move = true, overrideAllow = false;
    m_turretMotor->Set(0);
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
        if (dPad[L]->Get()) {
            x = ballLoadX;
            y = ballLoadHeight; 
        } else if (dPad[R]->Get()) {
            x = defaultX;
            y = discLoadHeight;
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
            x = rocketTopHeightBallX;
            y = rocketBallTopHeight;
        } else {
            move = false;
        }
    } else if (xbox->GetTriggerAxis(GenericHID::JoystickHand::kRightHand) > .1) {
        x = startPositionX;
        y = startPositionY;
        startPosition = true;
        turretPosition = TURRET_CENTER;
    } else {
        move = false;
        float shoulderAdd = (DeadZone(xbox->GetY(GenericHID::JoystickHand::kLeftHand), .4) * .02),
              elbowAdd = (DeadZone(xbox->GetY(GenericHID::JoystickHand::kRightHand), .4) * -.01);
        shoulderAngle += shoulderAdd;
        elbowAngle += elbowAdd;
        if (shoulderAdd > 0 && elbowAdd == 0) {
            overrideAllow = true;
        }
    }
    if (move) {
        float shoulderAngleMem = shoulderAngle, elbowAngleMem = elbowAngle;
        moveToPosition(x, y);
        if (((shoulderAngle + elbowAngle - M_PI > 0 && shoulderAngleMem + elbowAngleMem - M_PI < 0) ||
            (shoulderAngle + elbowAngle - M_PI < 0 && shoulderAngleMem + elbowAngleMem - M_PI > 0)) &&
            shoulderAngleMem < SAFE_SHOULDER_ANGLE) {
                shoulderOverride = true;
            }
    } else {
        if (xbox->GetBackButton()) {
            turretPosition = TURRET_LEFT;
        } else if (xbox->GetStartButton()) {
            turretPosition = TURRET_RIGHT;
        } else if (xbox->GetStickButton(GenericHID::JoystickHand::kRightHand)) {
            turretPosition = TURRET_CENTER;
        } else {
            if (turretMove != 0) {
                turretPosition = TURRET_NONE;
                m_turretMotor->Set(turretMove);
            }
        }
        #ifdef USE_LIDAR
            if(xbox->GetAButton()) { //This is for squaring off the turret
                m_turretMotor->SetSelectedSensorPosition(FindAngle(microLidar->GetMeasurement(2), microLidar->GetMeasurement(3))); // If right side
            } else if(xbox->GetBButton()) { //This is for squaring off the turret
                m_turretMotor->SetSelectedSensorPosition(FindAngle(microLidar->GetMeasurement(0), microLidar->GetMeasurement(1))); // If front
            } else if(xbox->GetXButton()) { //This is for squaring off the turret
                m_turretMotor->SetSelectedSensorPosition(FindAngle(microLidar->GetMeasurement(4), microLidar->GetMeasurement(5))); // If left side
            }
        #endif
        SmartDashboard::PutNumber("turret move", turretMove);
        SmartDashboard::PutNumber("turret position", turretPosition);
    }
    SetMotors(overrideAllow);
    init = false;
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
        // std::cout << "MOVING TO: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    } else {
        // std::cout << "INVALID ANGLE: x = " << x << ", y = " << y << ", ang1 = " << ang1 << ", ang2 = " << ang2 << "\n";
    }
}

double
Arm::computeElbowAngle()
{
#ifdef RED_BOT
    return -.00572901 * m_elbowMotor->GetSelectedSensorPosition(0) + 3.94251;
#else
    return -.00591263 * m_elbowMotor->GetSelectedSensorPosition(0) + 3.954;
#endif
}

double
Arm::computeShoulderAngle() {
#ifdef RED_BOT
    return 5.9427 * m_shoulderPot->Get() - 1.43229;
#else
    return 5.95463 * m_shoulderPot->Get() - .0799629;
#endif
}

double Arm::computeTurretAngle() {
#ifdef RED_BOT
    return -0.00576312 * m_turretMotor->GetSelectedSensorPosition(0) + 4.1677;
#else
    return -0.0413272 * m_turretMotor->GetSelectedSensorPosition(0) + 39.8119;
#endif
}

double
Arm::computeElbowPosition(double angle)
{
#ifdef RED_BOT
    return -173.267 * angle + 681.074;
#else
    // need BLACK_BOT numbers...for now if defi-ned using red
    return -169.12 * angle + 668.72; //input angle and output pot values 
    // -.00591263 * pot + 3.954      -> input pot values and output angle
    //Old Numbers: -169.284 * angle + 655.854
#endif
}

bool
Arm::validElbowPosition(double pos)
{
#ifdef RED_BOT
    if((pos < 100.0) || (pos > 700.0)) { // Original values were 200 and 600
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
    return 0.166743 * angle + 0.251658;
    // Old numbers: angle * 0.0837496 + 0.393355
#else // BLACK_BOT
    return .167931 * angle + .134296; // input angle output pot values
#endif
}
bool
Arm::validShoulderPosition(double pos)
{
    // need to figure out valid values for red and black bots
    return true;
}


bool
Arm::Within30InchLimit(float turretAngle) {
    float xTry = abs(armBaseFrontX / cos(abs(turretAngle))),
          yTry = abs(armBaseSideX / cos(M_PI / 2 - abs(turretAngle))),
	      x0 = lowArmLength * cos(computeShoulderAngle()) + highArmLength
            * cos(computeShoulderAngle() + computeElbowAngle() - M_PI) + clawLength;
	if (xTry < yTry) {
        SmartDashboard::PutNumber("CALC_X", (x0 * cos(abs(turretAngle)) - armBaseFrontX + turretOffset) / 25.4);
		return (x0 * cos(abs(turretAngle)) - armBaseFrontX + turretOffset) < 711;
	} else {
        SmartDashboard::PutNumber("CALC_X", (x0 * cos(M_PI / 2 - abs(turretAngle)) - armBaseSideX + turretOffset) / 25.4);
		return (x0 * cos(M_PI / 2 - abs(turretAngle)) - armBaseSideX + turretOffset) < 711;
	}
}

void
Arm::SetMotors(float overrideAllow)
{
    double elbowPosition;
    double shoulderPosition;
    // Compute the position that the elbow and shoulder potentiometers should
    // be at and tell the motorcontroller to go there via the PID closed loop.
    // The talon can do this itself, for the shoulder motor controller we need
    // to do the PID control loop in software.  Elbow has a fairly large error
    // which varies over the range +/- 20 units. Shoulder moves slowly to it's
    // position, which may or may not be an issue.
    
    //if we are out of bounds initially, comment out this whole loop then re-deploy
    if (!overrideAllow && !init) {
        if (Within30InchLimit(computeTurretAngle())) {
            elbowAngleTestMem = computeElbowAngle();
            shoulderAngleTestMem = computeShoulderAngle();
        } else {
            shoulderAngle = shoulderAngleTestMem;
            elbowAngle = elbowAngleTestMem;
        }
    }
    float yHeight = armBaseHeight + lowArmLength * sin(shoulderAngle) + highArmLength * sin(shoulderAngle + elbowAngle - M_PI);
    if (startPosition) {
        // if we cannot move to start position safely
        if (abs(m_turretMotor->GetSelectedSensorPosition(0) - TURRET_CENTER) > 35 && (yHeight < yClearance || (curY == yClearance + 125 && curX == 150 && abs(computeShoulderPosition(shoulderAngle) - m_shoulderPot->Get()) >= .001))) {
            curX = 150;
            curY = yClearance + 125;
            moveToPosition(curX, curY);
            m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, computeElbowPosition(elbowAngle));
            HardPID(m_shoulderMotor, m_shoulderPot->Get(), computeShoulderPosition(shoulderAngle), .01, .001);
        } else {
            if (HardPID(m_turretMotor, m_turretMotor->GetSelectedSensorPosition(0), TURRET_CENTER, 20, 5)) {
                startPosition = false;
                startPositionReal = true;
                curX = startPositionX;
                curY = startPositionY;
                moveToPosition(curX, curY);
            }
        }
    } else {
        if (startPositionReal) {
            if (HardPID(m_shoulderMotor, m_shoulderPot->Get(), computeShoulderPosition(shoulderAngle), .005, .001)) {
                m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, computeElbowPosition(elbowAngle));
                startPositionReal = false;
            }
        } else {
            elbowPosition = computeElbowPosition(elbowAngle);
            shoulderPosition = computeShoulderPosition(shoulderAngle);
            if (shoulderOverride && !init) {
                if (HardPID(m_shoulderMotor, m_shoulderPot->Get(), computeShoulderPosition(SAFE_SHOULDER_ANGLE), .005, .001)) {
                    m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, elbowPosition);
                    shoulderOverride = false;
                }
            } else {
                if(validElbowPosition(elbowPosition)) {
                    m_elbowMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, elbowPosition);
                }
                if(validShoulderPosition(shoulderPosition)) {
                    HardPID(m_shoulderMotor, m_shoulderPot->Get(), shoulderPosition, .005, .001);
                    // m_shoulderController->SetSetpoint(shoulderPosition);
                    // m_shoulderController->SetEnabled(true);
                }
            }
            if (turretPosition != TURRET_NONE) {
                HardPID(m_turretMotor, m_turretMotor->GetSelectedSensorPosition(0), turretPosition, 20, 5);
                // m_turretMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, turretPosition);
            }
        }
    }
}

// this function takes in the x distance from the target 
// starting from the edge of the drive train, and the y
// from the ground, and computes the required arm angles.
bool
Arm::FindArmAngles(float x, float y, float *ang1, float *ang2)
{
    //TBD: must make the x value vary based on where the turret is.
    // for now i assume it is facing forward
	y -= armBaseHeight;
    x += armBaseFrontX - turretOffset;
    float r = sqrt(x*x+y*y);
	*ang2 = acos((highArmLength * highArmLength + lowArmLength * lowArmLength - r * r) / (2 * highArmLength * lowArmLength));
	*ang1 = acos((lowArmLength * lowArmLength + r * r - highArmLength * highArmLength) / (2 * lowArmLength * r)) + atan(y / x);
    // HERE must find out whether or not the angles are allowed on the robot and return true or false accordingly
    return (*ang1 > 0 && *ang2 > 0);
}

bool Arm::HardPID(CANSparkMax *motor, float currentPosition, float finalPosition, float fastThreshold, float slowThreshold) {
    if (abs(currentPosition - finalPosition) > fastThreshold) {
        if (currentPosition > finalPosition) {
            motor->Set(-1);
        } else {
            motor->Set(1);
        }
    } else {
        if (abs(currentPosition - finalPosition) > slowThreshold) {
            if (currentPosition > finalPosition) {
                motor->Set(-.1);
            } else {
                motor->Set(.1);
            }
        } else {
            motor->Set(0);
            return true;
        }
    }
    return false;
}

// black one is wackadoo... i think the potentiometer is messed up because its range is much smaller
bool
Arm::HardPID(WPI_TalonSRX *motor, float currentPosition, float finalPosition, float fastThreshold, float slowThreshold) {
    if (abs(currentPosition - finalPosition) > fastThreshold) {
        if (currentPosition > finalPosition) {
            motor->Set(.8);
        } else {
            motor->Set(-.8);
        }
    } else {
        if (abs(currentPosition - finalPosition) > slowThreshold) {
            if (currentPosition > finalPosition) {
                motor->Set(.2);
            } else {
                motor->Set(-.2);
            }
        } else {
            motor->Set(0);
            return true;
        }
    }
    return false;
}

float Arm::FindAngle(int frontSensor, int rearSensor){
    #ifdef USE_LIDAR
        float angle;

        if(frontSensor > rearSensor) {
            angle = M_PI / 2 + atan((frontSensor - rearSensor) / sensorFrontToBack);
        }
        else if(rearSensor > frontSensor) {
            angle = M_PI / 2 - atan((rearSensor - frontSensor) / sensorFrontToBack);
        }
        SmartDashboard::PutNumber("Angle", angle); //Testing only
        return angle;
    #endif
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
    SmartDashboard::PutNumber("Turret position", m_turretMotor->GetSelectedSensorPosition(0));

    SmartDashboard::PutNumber("Elbow position", m_elbowMotor->GetSelectedSensorPosition(0));
    SmartDashboard::PutNumber("Elbow close loop error", m_elbowMotor->GetClosedLoopError(0));
 
    SmartDashboard::PutNumber("Shoulder position", m_shoulderPot->Get());
    // SmartDashboard::PutNumber("Shoulder close loop error", m_shoulderController->GetError());
    SmartDashboard::PutNumber("Shoulder Error", abs(computeShoulderPosition(shoulderAngle) - m_shoulderPot->Get()));
    SmartDashboard::PutNumber("Turret Error", abs(m_turretMotor->GetSelectedSensorPosition(0) - turretPosition));

    SmartDashboard::PutNumber("Turret Angle", computeTurretAngle() * 180.0 / M_PI);
    SmartDashboard::PutNumber("Shoulder Angle", computeShoulderAngle() * 180.0 / M_PI);
    SmartDashboard::PutNumber("Elbow Angle", computeElbowAngle() * 180.0 / M_PI);
    SmartDashboard::PutNumber("Preset X", curX / 25.4);
    SmartDashboard::PutNumber("Preset Y", curY / 25.4);
    SmartDashboard::PutNumber("Preset Shoulder", shoulderAngle * 180 / M_PI);
    SmartDashboard::PutNumber("Preset Elbow", elbowAngle * 180 / M_PI);
    SmartDashboard::PutBoolean("Within 30\" range?", Within30InchLimit(computeTurretAngle()));
}
