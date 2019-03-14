#include <Drake.h>
#include <frc/JoyStick.h>

using namespace frc;
using namespace rev;

DalekDrive::DalekDrive(int leftFront, int leftRear,
         int rightFront, int rightRear, DalekDrive::driveType t)
{
	m_leftMotor[FRONT]   = new CANSparkMax(leftFront, CANSparkMax::MotorType::kBrushless);
	m_rightMotor[FRONT]  = new CANSparkMax(rightFront, CANSparkMax::MotorType::kBrushless);
	m_leftMotor[REAR]    = new CANSparkMax(leftRear, CANSparkMax::MotorType::kBrushless);
	m_rightMotor[REAR]   = new CANSparkMax(rightRear, CANSparkMax::MotorType::kBrushless);
	m_left               = new SpeedControllerGroup(*m_leftMotor[FRONT], *m_leftMotor[REAR]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	m_type               = t;
	if(t == DalekDrive::driveType::kDifferential) {
		m_diffdrive      = new DifferentialDrive(*m_left, *m_right);
	}
	else {
	    m_mecanum        = new MecanumDrive(*m_leftMotor[FRONT], *m_leftMotor[REAR],
	 							*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	}
	m_leftEncoder[FRONT] = new CANEncoder(*m_leftMotor[FRONT]);
	m_leftEncoder[REAR]  = new CANEncoder(*m_leftMotor[REAR]);
	m_rightEncoder[FRONT]= new CANEncoder(*m_rightMotor[FRONT]);
	m_rightEncoder[REAR] = new CANEncoder(*m_rightMotor[REAR]);
	m_needFree           = true;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax* leftFront, CANSparkMax* leftRear,
         CANSparkMax* rightFront, CANSparkMax* rightRear, DalekDrive::driveType t)
{
	m_leftMotor[FRONT]   = leftFront;
	m_rightMotor[FRONT]  = rightFront;
	m_leftMotor[REAR]    = leftRear;
	m_rightMotor[REAR]   = rightRear;    	
	m_left               = new SpeedControllerGroup(*m_leftMotor[FRONT], *m_leftMotor[REAR]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	m_type               = t;
	if(t == DalekDrive::driveType::kDifferential) {
		m_diffdrive      = new DifferentialDrive(*m_left, *m_right);
	}
	else {   
		m_mecanum        = new MecanumDrive(*m_leftMotor[FRONT], *m_leftMotor[REAR],
	 							*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	}
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::DalekDrive(CANSparkMax& leftFront, CANSparkMax& leftRear,
         CANSparkMax& rightFront, CANSparkMax& rightRear, DalekDrive::driveType t)
{
	m_leftMotor[FRONT]   = &leftFront;
	m_rightMotor[FRONT]  = &rightFront;
	m_leftMotor[REAR]    = &leftRear;
	m_rightMotor[REAR]   = &rightRear;
    m_left               = new SpeedControllerGroup(*m_leftMotor[FRONT], *m_leftMotor[REAR]);
    m_right              = new SpeedControllerGroup(*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	m_type               = t;
	if(m_type == DalekDrive::driveType::kDifferential) {
		m_diffdrive      = new DifferentialDrive(*m_left, *m_right);
	}
	else {
		m_mecanum        = new MecanumDrive(*m_leftMotor[FRONT], *m_leftMotor[REAR],
	 							*m_rightMotor[FRONT], *m_rightMotor[REAR]);
	}
	m_needFree           = false;
	InitDalekDrive();
}

DalekDrive::~DalekDrive()
{
	if(m_type == DalekDrive::driveType::kDifferential)
		delete m_diffdrive;
	else
		delete m_mecanum;
    delete m_left;
    delete m_right;
	if(m_needFree) {
		delete m_leftMotor[FRONT];
		delete m_rightMotor[FRONT];
		delete m_leftMotor[REAR];
		delete m_rightMotor[REAR];
	}
	m_needFree = false;
	return;
}

double
DalekDrive::squareInput(double v)
{
	// sign preserving square function
	if (v < 0.0)
		return -(v * v);
	return (v * v);
}

void
DalekDrive::TankDrive(frc::Joystick* leftStick, frc::Joystick* rightStick,
             bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->TankDrive(leftStick->GetY(), rightStick->GetY(), squaredInputs);
	else {
		// note mecanum wheels aren't really meant to run this way but can.  As such
		// it's not supported by the MecanumDrive class, so hacked up tank drive
		double l, r;

		l = leftStick->GetY(); r = rightStick->GetY();
		if(squaredInputs) {
			l = squareInput(l);
			r = squareInput(r);
		}
		m_leftMotor[FRONT]->Set(l);  m_leftMotor[REAR]->Set(l);
		m_rightMotor[FRONT]->Set(r); m_rightMotor[REAR]->Set(r);
	}
}

void
DalekDrive::TankDrive(frc::Joystick& leftStick, frc::Joystick& rightStick,
             bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->TankDrive(leftStick.GetY(), rightStick.GetY(), squaredInputs);	
	else {
		double l, r;

		l = leftStick.GetY(); r = rightStick.GetY();
		if(squaredInputs) {
			l = squareInput(l);
			r = squareInput(r);
		}
		m_leftMotor[FRONT]->Set(l);  m_leftMotor[REAR]->Set(l);
		m_rightMotor[FRONT]->Set(r); m_rightMotor[REAR]->Set(r);
	}
}

void
DalekDrive::TankDrive(double leftValue, double rightValue, bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->TankDrive(leftValue, rightValue, squaredInputs);
	else {
		double l, r;

		l = leftValue; r = rightValue;
		if(squaredInputs) {
			l = squareInput(l);
			r = squareInput(r);
		}
		m_leftMotor[FRONT]->Set(l);  m_leftMotor[REAR]->Set(l);
		m_rightMotor[FRONT]->Set(r); m_rightMotor[REAR]->Set(r);
	}
}

void
DalekDrive::ArcadeDrive(frc::Joystick* stick, bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->ArcadeDrive(stick->GetX(), stick->GetY(), squaredInputs);
}

void
DalekDrive::ArcadeDrive(frc::Joystick& stick, bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->ArcadeDrive(stick.GetX(), stick.GetY(), squaredInputs);
}

void
DalekDrive::ArcadeDrive(double moveValue, double rotateValue, bool squaredInputs)
{
	if(m_type == DalekDrive::driveType::kDifferential)
		m_diffdrive->ArcadeDrive(moveValue, rotateValue, squaredInputs);
}

void
DalekDrive::SetLeftRightMotorOutputs(double leftOutput, double rightOutput)
{
	m_leftMotor[FRONT]->Set(leftOutput); m_rightMotor[FRONT]->Set(rightOutput);
	m_leftMotor[REAR]->Set(leftOutput);  m_rightMotor[REAR]->Set(rightOutput);
}

void 
DalekDrive::Polar(frc::Joystick* stick)
{
	if(m_type == DalekDrive::driveType::kMecanum) {
		m_mecanum->DrivePolar(stick->GetY(), stick->GetX(), stick->GetTwist());
	}
}

void
DalekDrive::Polar(frc::Joystick& stick)
{
	if(m_type == DalekDrive::driveType::kMecanum) {
		m_mecanum->DrivePolar(stick.GetY(), stick.GetX(), stick.GetTwist());
	}
}
	
void
DalekDrive::Polar(double magnitude, double angle, double zRotation)
{
	if(m_type == DalekDrive::driveType::kMecanum) {
		m_mecanum->DrivePolar(magnitude, angle, zRotation);
	}
}

void
DalekDrive::Cartesian(frc::Joystick* stick,	double gyroAngle)
{
	if(m_type == DalekDrive::driveType::kMecanum) {
		double x, y, z;
		x = stick->GetX(); x = squareInput(DeadZone(x, .1));
		y = stick->GetY(); y = squareInput(DeadZone(y, .1));
		z = stick->GetTwist(); z = squareInput(squareInput(DeadZone(z, .1))) / 5;
		// if (stick->GetButton(Joystick::ButtonType::kTriggerButton)) {
		// 	x /= 1000;
		// 	y /= 1000;
		// 	z /= 1000;
		// 	cout << "SHOULD BE LOWERING\n";
		// }
//		z = z + x + y;
//		if(z > 1){
//			z = 1;
//		}
		m_mecanum->DriveCartesian(-x, y, -z, gyroAngle);
	}
}

void 
DalekDrive::Cartesian(frc::Joystick& stick, double gyroAngle)
{
	// for now I'm leaving this the way that was done.  If you want to continue
	// to have the Cartesian have so much dead space and non linearity, change the
	// above function to use the same calculations.  Note: This method isn't 
	// currently used
	if(m_type == DalekDrive::driveType::kMecanum) {
		m_mecanum->DriveCartesian(DeadZone(stick.GetX(), .3) * .5, DeadZone(stick.GetY(), .3) * -.5, DeadZone(stick.GetTwist(), .3) * .2,
			gyroAngle);
	}
}

void 
DalekDrive::Cartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle)
{
	if(m_type == DalekDrive::driveType::kMecanum) {
		m_mecanum->DriveCartesian(ySpeed, xSpeed, zRotation, gyroAngle);
	}
}
	
void
DalekDrive::SetInvertedMotor(int side, bool isInverted)
{
	switch(side) {
	case LEFT:
		m_leftMotor[FRONT]->SetInverted(isInverted);
		m_leftMotor[REAR]->SetInverted(isInverted);
		break;

	case RIGHT:
		m_rightMotor[FRONT]->SetInverted(isInverted);
		m_rightMotor[REAR]->SetInverted(isInverted);
		break;

	default:
		break;
	}
}

void
DalekDrive::InitDalekDrive(void)
{
	// Setup Encoder access
	m_leftEncoder[FRONT] = new CANEncoder(*m_leftMotor[FRONT]);
	m_leftEncoder[REAR]  = new CANEncoder(*m_leftMotor[REAR]);
	m_rightEncoder[FRONT]= new CANEncoder(*m_rightMotor[FRONT]);
	m_rightEncoder[REAR] = new CANEncoder(*m_rightMotor[REAR]);

	// Configure the SparkMax
    m_leftMotor[FRONT]->SetCANTimeout(CAN_TIMEOUT);
	m_leftMotor[FRONT]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_leftMotor[FRONT]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_leftMotor[FRONT]->SetOpenLoopRampRate(RAMP_RATE);                  // used to be SetRampRate... update i guess... check if this is oprn or closed
	m_leftMotor[FRONT]->SetInverted(true);

    m_rightMotor[FRONT]->SetCANTimeout(CAN_TIMEOUT);
	m_rightMotor[FRONT]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_rightMotor[FRONT]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_rightMotor[FRONT]->SetOpenLoopRampRate(RAMP_RATE);                // same
	m_rightMotor[FRONT]->SetInverted(true);

    m_leftMotor[REAR]->SetCANTimeout(CAN_TIMEOUT);
  	m_leftMotor[REAR]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_leftMotor[REAR]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_leftMotor[REAR]->SetOpenLoopRampRate(RAMP_RATE);                  // same
	m_leftMotor[REAR]->SetInverted(true);

    m_rightMotor[REAR]->SetCANTimeout(CAN_TIMEOUT);
	m_rightMotor[REAR]->SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_rightMotor[REAR]->SetSmartCurrentLimit(STALL_LIMIT, FREE_LIMIT, 0);
	m_rightMotor[REAR]->SetOpenLoopRampRate(RAMP_RATE);                 // same
	m_rightMotor[REAR]->SetInverted(true);

    m_leftMotor[FRONT]->StopMotor();  m_leftMotor[REAR]->StopMotor();
    m_rightMotor[FRONT]->StopMotor(); m_rightMotor[REAR]->StopMotor();
}

float
DalekDrive::DeadZone(float input, float range) {
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
DalekDrive::printFaults(int p, int faults)
{
    switch(p) {
	case LEFT: 
		frc::SmartDashboard::PutNumber("Left Drive Motor reported faults", faults);
		if(m_leftMotor[REAR]) {
			frc::SmartDashboard::PutNumber("Left slave status",
					m_leftMotor[REAR]->GetFaults());
		}
        break; 
    case RIGHT:
		frc::SmartDashboard::PutNumber("Right Drive Motor reported faults", faults);
		if(m_rightMotor[REAR]) {
			frc::SmartDashboard::PutNumber("Right slave status",
					m_rightMotor[REAR]->GetFaults());
		}
        break;
    default:
        break;
    }
	return;
}

bool
DalekDrive::DriveOk()
{
	int mstat;

	// update dashboard of current draw for motors
	frc::SmartDashboard::PutNumber("Left Front current", 
		m_leftMotor[FRONT]->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Left Front Encoder position",
		m_leftEncoder[FRONT]->GetPosition());
	frc::SmartDashboard::PutNumber("Left Rear Motor current",
		m_leftMotor[REAR]->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Left Rear Encoder position",
		m_leftEncoder[REAR]->GetPosition());

	frc::SmartDashboard::PutNumber("Right Front current", 
		m_rightMotor[FRONT]->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Right Front Encoder position",
		m_rightEncoder[FRONT]->GetPosition());
	frc::SmartDashboard::PutNumber("Right Rear Motor current",
		m_rightMotor[REAR]->GetOutputCurrent());
	frc::SmartDashboard::PutNumber("Right Rear Encoder position",
		m_rightEncoder[REAR]->GetPosition());

	// check for motor faults
	mstat = m_leftMotor[FRONT]->GetFaults();
	if(mstat != 0) {
		printFaults(LEFT, mstat);
		return false;
	}
	mstat = m_leftMotor[FRONT]->GetStickyFaults();
	if(mstat) {
		printFaults(LEFT, mstat);
		return false;
	}

	mstat = m_rightMotor[FRONT]->GetFaults();
	if(mstat) {
		printFaults(RIGHT, mstat);
		return false;
	}
	mstat = m_rightMotor[FRONT]->GetStickyFaults();
	if(mstat) {
		printFaults(RIGHT, mstat);
		return false;
	}
	return true;
}
