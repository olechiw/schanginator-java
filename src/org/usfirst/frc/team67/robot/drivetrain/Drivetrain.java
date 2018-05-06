package org.usfirst.frc.team67.robot.drivetrain;

import org.usfirst.frc.team67.robotutils.ThreePointInterpolation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain
{
	Drivetrain()
	{
		m_rampMotor = new VictorSPX(Constants.RAMP_VICTOR);
		m_gyro = new PigeonIMU(Constants.PIGEON_CAN);
		// m_drive(m_lDriveF, m_rDriveF),
		m_distancePIDWrapper = new DistancePIDWrapper(this);
		m_anglePIDWrapper = new AnglePIDWrapper(this);
		m_distancePID = new PIDController(Constants.DISTANCE_DRIVE_P, Constants.DISTANCE_DRIVE_I,
				Constants.DISTANCE_DRIVE_D, m_distancePIDWrapper, m_distancePIDWrapper, 0.05);
		m_anglePID = new PIDController(Constants.ANGLE_DRIVE_P, Constants.ANGLE_DRIVE_I,
				Constants.ANGLE_DRIVE_D, m_anglePIDWrapper, m_anglePIDWrapper, 0.05);
		m_shift = new Solenoid(Constants.SOLENOID_SHIFT);
		m_ratchet = new Servo(Constants.SERVO_RATCHET);
		m_lDriveF = new WPI_TalonSRX(Constants.TALON_DRIVE_LF);
		m_lDriveM = new WPI_VictorSPX(Constants.VICTOR_DRIVE_LM);
		m_lDriveR = new WPI_VictorSPX(Constants.VICTOR_DRIVE_LR);

		m_rDriveF = new WPI_TalonSRX(Constants.TALON_DRIVE_RF);
		m_rDriveM = new WPI_VictorSPX(Constants.VICTOR_DRIVE_RM);
		m_rDriveR = new WPI_VictorSPX(Constants.VICTOR_DRIVE_RR);

		m_drive = new DifferentialDrive(m_lDriveF, m_rDriveF);

		m_sonicLeft = new AnalogInput(Constants.ULTRA_LEFT);
		m_sonicRight = new AnalogInput(Constants.ULTRA_RIGHT);
		m_sonicRear = new AnalogInput(Constants.ULTRA_REAR);

		m_turn = 0;
		m_speed = 0;

		// m_drive = new DifferentialDrive(m_lDriveF, m_rDriveF);

		m_lDriveF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_lDriveF.setInverted(false);

		m_lDriveM.follow(m_lDriveF);
		m_lDriveM.setInverted(false);

		m_lDriveR.follow(m_lDriveF);
		m_lDriveR.setInverted(false);

		m_rDriveF.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_rDriveF.setInverted(false);

		m_rDriveM.follow(m_rDriveF);
		m_rDriveM.setInverted(false);

		m_rDriveR.follow(m_rDriveF);
		m_rDriveR.setInverted(false);

		sweetTurnMaxPct = new ThreePointInterpolation(Constants.SweetTurnMaxPct);
		sweetTurnRampDownStart = new ThreePointInterpolation(Constants.SweetTurnRampDownStartOffSet);
		sweetTurnRampDownRate = new ThreePointInterpolation(Constants.SweetTurnRampDownRate);

	}

	// ArcadeDrive
	public void ArcadeDrive(double speed, double angle)
	{
		m_drive.setSafetyEnabled(false);
		// m_drive.SetExpiration(1000);
		m_speed = speed;
		m_turn = angle;
		m_drive.arcadeDrive(speed, angle, false);

	}

	public void SetTurn(double turn)
	{
		ArcadeDrive(m_speed, turn);
	}

	public void SetSpeed(double speed)
	{
		ArcadeDrive(speed, m_turn);
	}

	public double GetTurn()
	{
		return m_turn;
	}

	public double GetSpeed()
	{
		return m_speed;
	}

	public double GetLeftTalonPercentOutput()
	{
		return m_lDriveF.getMotorOutputPercent();
	}

	public double GetRightTalonPercentOutput()
	{
		return m_rDriveF.getMotorOutputPercent();
	}

	public double GetRampPercentOutput()
	{
		return m_rampMotor.getMotorOutputPercent();
	}

	// Update PID
	public void UpdateDrivePID()
	{
		SetAnglePID(Constants.ANGLE_DRIVE_P, Constants.ANGLE_DRIVE_I,
				Constants.ANGLE_DRIVE_D);
		SetDistancePID(Constants.DISTANCE_DRIVE_P, Constants.DISTANCE_DRIVE_I,
				Constants.DISTANCE_DRIVE_D);

	}

	public void UpdateDriveHIPID()
	{
		SetAnglePID(Constants.ANGLE_DRIVEHI_P, Constants.ANGLE_DRIVEHI_I,
				Constants.ANGLE_DRIVEHI_D);
		SetDistancePID(Constants.DISTANCE_DRIVEHI_P, Constants.DISTANCE_DRIVEHI_I,
				Constants.DISTANCE_DRIVEHI_D);
	}

	public void UpdateTurnPID()
	{
		SetAnglePID(Constants.ANGLE_TURN_P, Constants.ANGLE_TURN_I,
				Constants.ANGLE_TURN_D);
		SetDistancePID(Constants.DISTANCE_TURN_P, Constants.DISTANCE_TURN_I,
				Constants.DISTANCE_TURN_D);
	}

	public void UpdateDrivePIDOutput(double output)
	{
		distancePIDSpeed = output;
	}

	public void UpdateTurnPIDOutput(double output)
	{
		turnPIDSpeed = output;
	}

	// Angle PID Controller
	public void EnableAngle()
	{
		m_anglePID.enable();
	}

	public boolean IsAngleEnabled()
	{
		return m_anglePID.isEnabled();
	}

	public void DisableAngle()
	{
		m_anglePID.disable();
	}

	public void SetAnglePID(double p, double i, double d)
	{
		m_anglePID.setPID(p, i, d);
	}

	void SetAngleSetPoint(double setpoint)
	{
		m_anglePID.setSetpoint(setpoint);
	}

	double GetAngleSetpoint()
	{
		return m_anglePID.getSetpoint();
	}

	public void ResetAnglePID()
	{
		m_anglePID.reset();
	}

	public double GetAnglePIDError()
	{
		return m_anglePID.getError();
	}

	public void SetRMotorController(double output)
	{
		m_rDriveF.set(ControlMode.PercentOutput, output);
	}

	public void SetLMotorController(double output)
	{
		m_lDriveF.set(ControlMode.PercentOutput, output);
	}

	// Distance PID Controller
	public void EnableDistance()
	{
		m_distancePID.enable();
	}

	public boolean IsDistanceEnabled()
	{
		return m_distancePID.isEnabled();
	}

	public void DisableDistance()
	{
		m_distancePID.disable();
	}

	public void SetDistancePID(double p, double i, double d)
	{
		m_distancePID.setPID(p, i, d);
	}

	public void SetDistanceSetPoint(double setpoint)
	{
		m_distancePID.setSetpoint(setpoint / Constants.TUNIT_CONVERSION);
	}

	public double GetDistanceSetpoint()
	{
		return m_distancePID.getSetpoint();
	}

	public void ResetDistancePID()
	{
		m_distancePID.reset();
	}

	public double GetDistancePIDError()
	{
		return m_distancePID.getError() * Constants.TUNIT_CONVERSION;
	}

	public void SetMotorControllerTest(double speed)
	{
		m_rDriveF.set(ControlMode.PercentOutput, speed);
	}

	// PID Controller
	public void EnablePID()
	{
		EnableDistance();
		EnableAngle();
	}

	public void SetPIDSetpoint(double distance, double angle)
	{
		SetDistanceSetPoint(distance);
		SetAngleSetPoint(angle);
	}

	public void DisablePID()
	{
		DisableDistance();
		DisableAngle();
	}

	public double GetDistancePIDSpeed()
	{
		return distancePIDSpeed;
	}

	// SWEET PROFILES
	public void SweetTurnRestart()
	{
		sweetTurnRate = 0;
		sweetTurnState = sweetTurnSt.sweetTurn_reset;
		sweetTurnTotalAngleTravel = 0;
		sweetTurnDirection = 0;
		sweetTurnTimer = 0;
		sweetTurnIterateCounter = 0;
	}

	public boolean SweetTurnFinished(double target, double MinErrorToExit, double maxSpeed)
	{

		boolean complete = false;
		double absError = Math.abs(target - YawPitchRoll[0]);
		double maxPct;
		double rampDownStart;
		double remainingAngleAtStartRampDown = 360;
		double xyz_dps[] = new double[3];

		m_gyro.getRawGyro(xyz_dps);

		if (sweetTurnIterateCounter > Constants.SWEET_TURN_ITERATE_MAX)
		{
			sweetTurnState = sweetTurnSt.sweetTurn_reset;
			sweetTurnIterateCounter = 0;
			complete = true;
		}

		if (sweetTurnState != sweetTurnSt.sweetTurn_reset && ((sweetTurnDirection == 1 && YawPitchRoll[0] > target)
				|| (sweetTurnDirection == -1 && YawPitchRoll[0] < target)))
		{
			sweetTurnState = sweetTurnSt.sweetTurn_reset;
			sweetTurnIterateCounter++;
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_reset)
		{
			sweetTurnRate = 0;
			sweetTurnTimer = 0;
			sweetTurnState = sweetTurnSt.sweetTurn_RampIn;
			sweetTurnTotalAngleTravel = absError;
			sweetTurnDirection = target > YawPitchRoll[0] ? 1 : -1;
		}

		maxPct = sweetTurnMaxPct.GetMappedValue(sweetTurnTotalAngleTravel);
		if (maxPct > maxSpeed)
		{
			maxPct = maxSpeed;
		}

		rampDownStart = sweetTurnRampDownStart.GetMappedValue(Math.abs(xyz_dps[2]));

		if (sweetTurnState == sweetTurnSt.sweetTurn_RampIn)
		{
			sweetTurnRate += Constants.SWEET_TURN_RAMP_UP_RATE;

			if (absError <= MinErrorToExit && Math.abs(xyz_dps[2]) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			} else if (absError <= rampDownStart)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_RampDown;
				remainingAngleAtStartRampDown = absError;
			} else if (sweetTurnRate >= maxPct)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Max;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_Max)
		{

			sweetTurnRate = maxPct;

			if (absError <= MinErrorToExit)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			} else if (absError <= rampDownStart)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_RampDown;
				remainingAngleAtStartRampDown = absError;
			} else if (sweetTurnRate >= maxPct)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Max;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_RampDown)
		{
			sweetTurnRate -= sweetTurnRampDownRate.GetMappedValue(remainingAngleAtStartRampDown);

			if (absError <= MinErrorToExit && Math.abs(xyz_dps[2]) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			} else if (sweetTurnRate <= Constants.SWEET_TURN_PERCISE_TURN_PCT)
			{
				sweetTurnState = sweetTurnSt.sweetTurn_Precision;
			}
		}

		if (sweetTurnState == sweetTurnSt.sweetTurn_Precision)
		{
			sweetTurnRate = Constants.SWEET_TURN_PERCISE_TURN_PCT;

			if (absError <= MinErrorToExit && Math.abs(xyz_dps[2]) <= Constants.SWEET_TURN_MAX_EXIT_VELOCITY)
			{
				sweetTurnRate = 0;
				sweetTurnTimer = 0;
				sweetTurnIterateCounter = 0;
				complete = true;
				sweetTurnState = sweetTurnSt.sweetTurn_reset;
			}
		}

		ArcadeDrive(0, -sweetTurnDirection * sweetTurnRate);
		return complete;

	}

	// Encoder
	public double GetRawLeftDistance()
	{
		return m_lDriveF.getSelectedSensorPosition(0);
	}

	public double GetRawRightDistance()
	{
		return m_rDriveF.getSelectedSensorPosition(0);
	}

	public double GetLeftDistance()
	{
		return m_lDriveF.getSelectedSensorPosition(0) * Constants.TUNIT_CONVERSION; // this was originally
																								// positive but
		// configured negative for 2018 robot
	}

	public double GetRightDistance()
	{
		return m_rDriveF.getSelectedSensorPosition(0) * -Constants.TUNIT_CONVERSION; // as was this
	}

	public double GetAverageDistance()
	{
		return ((GetLeftDistance() + GetRightDistance()) / 2);
	}

	public void ZeroDistance()
	{
		m_lDriveF.setSelectedSensorPosition(0, 0, 0);
		m_rDriveF.setSelectedSensorPosition(0, 0, 0);
	}

	// Gyro
	public double GetYaw()
	{
		PreviousYawPitchRoll[0] = YawPitchRoll[0];
		m_gyro.getYawPitchRoll(YawPitchRoll);
		DiffYawPitchRoll[0] = YawPitchRoll[0] - PreviousYawPitchRoll[0];

		if (Math.abs(DiffYawPitchRoll[0]) > Constants.PIGEON_RESET)
		{
			badPigeonCounter++;
			m_gyro.setYaw(PreviousYawPitchRoll[0], 0);
			m_gyro.getYawPitchRoll(YawPitchRoll);
			return PreviousYawPitchRoll[0];
		} else
		{
			return YawPitchRoll[0];
		}

	}

	public double GetYawRate()
	{
		double xyz_dps[] = new double[3];

		m_gyro.getRawGyro(xyz_dps);

		return xyz_dps[2];

	}

	public void ZeroGyro()
	{
		m_gyro.setYaw(0, 10);
	}

	public boolean HasGyroReset()
	{
		return m_gyro.hasResetOccurred();
	}

	public int PigeonState()
	{

		return m_gyro.getState().value;
	}

	// Talon Data
	public double DisplayPercentOutputLeft()
	{
		return m_lDriveF.get();
	}

	public double DisplayPercentOutputRight()
	{
		return m_rDriveF.get();
	}

	// Ramp
	public void SetRamp(double j)
	{
		if (j > Constants.RAMP_DEADBAND)
		{
			m_rampMotor.set(ControlMode.PercentOutput, Constants.RAMP_SPEED);
		} else if (j < -Constants.RAMP_DEADBAND)
		{
			m_rampMotor.set(ControlMode.PercentOutput, -Constants.RAMP_SPEED);
		} else
		{
			m_rampMotor.set(ControlMode.PercentOutput, 0.0);
		}
		// j = joystick axis
	}

	public boolean HasRampReset()
	{
		return m_rampMotor.hasResetOccurred();
	}

	public double GetRampSpeed()
	{
		return Constants.RAMP_SPEED;
	}

	// Gear Shift
	public void SetShift(boolean shift)
	{
		m_shift.set(shift);
	}

	public void SetRatchet(double position)
	{
		m_ratchet.set(position);
	}

	public int badPigeonCounter = 0;
	public double turnPIDSpeed = 0.8;
	public double distancePIDSpeed = 0.80; // multiplier to make PID calm down
	public double distancePIDOutput = 0;
	public double anglePIDOutput = 0;

	public double[] YawPitchRoll;
	public double[] PreviousYawPitchRoll;
	public double[] DiffYawPitchRoll;

	// UltraSonicsss
	public double GetUltrasonicLeft()
	{
		leftSonicDistance = m_sonicLeft.getAverageVoltage() * Constants.ULTRA_VOLTS_TO_INCHES;
		return leftSonicDistance;
	}

	public double GetUltrasonicRight()
	{
		rightSonicDistance = m_sonicRight.getAverageVoltage() * Constants.ULTRA_VOLTS_TO_INCHES;
		return rightSonicDistance;

	}

	public double GetUltrasonicRear()
	{
		rearSonicDistance = m_sonicRear.getAverageVoltage() * Constants.ULTRA_VOLTS_TO_INCHES;
		return rearSonicDistance;

	}

	public double leftSonicDistance;
	public double rightSonicDistance;
	public double rearSonicDistance;

	// Mopro Turns
	public void magicalTurn(double angle) // desired angle of turn, positive is right
	{
		// encoder ticks per 1 wheel revolution
		// total circumference divided by wheel circumference equals circumference
		// covered by one wheel rotation
		// total circumference divided by three sixty equals circumference covered per
		// degree
		// circ/onewheelrev(this is wheel rotations per 360 degrees) / circ/degree (this
		// is total circumference covered per degree) = degree/onewheelrev IGNORE THIS
		// circ/onewheelrev(this is wheel rotations per 360 degrees) / 360 = wheel
		// rotations per degree
		// convert to encoder ticks and that is input number

		// wheel diameter = 4 inches wheel circumference 12.56636
		// total diameter = 11.475 total circumference 72.0994905
		// encoder ticks per 1 wheel revolution = 21168.6741
		// ONE_WHEEL_ROTATION_CIRCUMFERENCE 5.7375/1 inches/onewheelrotation
		// diameter in inches per one degree 0.2002763625 inches/onedegree
		// wheel rotations per degree 0.0349065556

		turnEncoder = 21168.6741 / 0.0349065556;
		m_lDriveF.set(ControlMode.MotionMagic, (angle * turnEncoder));
		m_rDriveF.set(ControlMode.MotionMagic, (angle * -turnEncoder));

	}

	public double turnEncoder = 0.0;

	private WPI_TalonSRX m_lDriveF;
	private WPI_VictorSPX m_lDriveM;
	private WPI_VictorSPX m_lDriveR;
	private WPI_TalonSRX m_rDriveF;
	private WPI_VictorSPX m_rDriveM;
	private WPI_VictorSPX m_rDriveR;

	private VictorSPX m_rampMotor;
	private PigeonIMU m_gyro;

	private DifferentialDrive m_drive;

	private DistancePIDWrapper m_distancePIDWrapper;
	private AnglePIDWrapper m_anglePIDWrapper;

	private PIDController m_distancePID;
	private PIDController m_anglePID;

	private Solenoid m_shift;
	private Servo m_ratchet;

	private AnalogInput m_sonicLeft;
	private AnalogInput m_sonicRight;
	private AnalogInput m_sonicRear;

	private double m_turn, m_speed;

	private enum sweetTurnSt
	{
		sweetTurn_reset, sweetTurn_RampIn, sweetTurn_Max, sweetTurn_RampDown, sweetTurn_Precision
	};

	private ThreePointInterpolation sweetTurnMaxPct;
	private ThreePointInterpolation sweetTurnRampDownStart;
	private ThreePointInterpolation sweetTurnRampDownRate;

	private double sweetTurnTimer;
	private sweetTurnSt sweetTurnState;
	private double sweetTurnRate;
	private double sweetTurnTotalAngleTravel;
	private double sweetTurnDirection; // should only ever be -1 or 1
	private double sweetTurnIterateCounter = 0;
}
