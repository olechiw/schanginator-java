package org.usfirst.frc.team67.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake
{
	public double WristAngle;
	public double DesiredWristPosition = 0;

	public double IntakeDesiredSpeed = 0;
	public double WristPastPosition = 0;
	public double WristCurrentPosition = 0;
	public double WristCommandedPosition = 0;

	public double OutputCurrent = 0;
	public double ClosedLoopError = 0;
	public double MotorOutputWrist = 0;
	public double MotorOutputMaster = 0;
	public double MotorOutputSlave = 0;

	public Intake()
	{
		m_intakeVictor = new VictorSPX(Constants.VICTOR_INTAKE_MASTER);
		m_slaveVictor = new VictorSPX(Constants.VICTOR_INTAKE_SLAVE);
		m_wrist = new TalonSRX(Constants.TALON_WRIST);
		m_claw = new Solenoid(Constants.CLAW_SOLENOID);
		m_neutralClaw = new Solenoid(Constants.CLAW_NEUTRAL_SOLENOID);

		m_rollingTimer = new Timer();
		m_spitTimer = new Timer();
		m_slaveVictor.follow(m_intakeVictor);
		m_intakeVictor.setInverted(true);

		m_intakeVictor.setNeutralMode(NeutralMode.Brake);
		m_slaveVictor.setNeutralMode(NeutralMode.Brake);

		m_wrist.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		m_wrist.selectProfileSlot(0, 0);

		m_wrist.configNominalOutputForward(0, 10);
		m_wrist.configNominalOutputForward(0, 10);
		m_wrist.configNominalOutputForward(1, 10);
		m_wrist.configNominalOutputReverse(-1, 10);

		m_wrist.config_kP(0, Constants.ANGLE_INTAKE_DOWN_P, 10);
		m_wrist.config_kI(0, Constants.ANGLE_INTAKE_DOWN_I, 10);
		m_wrist.config_kD(0, Constants.ANGLE_INTAKE_DOWN_D, 10);
		m_wrist.config_kF(0, Constants.ANGLE_INTAKE_DOWN_F, 10);

		m_wrist.configMotionCruiseVelocity(400, 10); // 200
		m_wrist.configMotionAcceleration(600, 10);

		m_wrist.setSelectedSensorPosition(0, 0, 10);
		// m_wrist.SetSensorPhase(true);
		// m_wrist.SetInverted(true);

		// topLimit = 1500; //TODO 90degrees
		// bottomLimit = -100;

	}

	public void ReadSensors()
	{
		OutputCurrent = m_wrist.getOutputCurrent();
		ClosedLoopError = m_wrist.getClosedLoopError(0);
		WristCurrentPosition = m_wrist.getSelectedSensorPosition(0);
		MotorOutputWrist = m_wrist.getMotorOutputPercent();
		MotorOutputMaster = m_intakeVictor.getMotorOutputPercent();
		MotorOutputSlave = m_slaveVictor.getMotorOutputPercent();
	}

	// Pneumatics
	public void OpenClaw(boolean open)
	{
		ClawOpen = open;
		m_claw.set(ClawOpen);

	}

	public void NeutralClaw(boolean open)
	{
		neutralClawOpen = open;
		m_neutralClaw.set(neutralClawOpen);

	}

	// Intake Motors
	public void RunIntake(double speed)
	{
		SmartDashboard.putNumber("Intake Case", m_intakeCase);
		SmartDashboard.putNumber("Intake Timer", m_rollingTimer.get());

		SmartDashboard.putNumber("Intake Spit Case", m_spitCase);
		prevIntakeSpeed = currentIntakeSpeed;
		currentIntakeSpeed = speed;

		if (speed < 0.0)
		{
			m_intakeVictor.setInverted(true);
			m_intakeVictor.set(ControlMode.PercentOutput, speed);
		} else if (speed > 0.0)
		{

			if (GetShoot() == false)
			{
				switch (m_spitCase)
				{
				case 0:

					spitIntakeSpeed = speed;
					m_intakeVictor.setInverted(true);
					m_intakeVictor.set(ControlMode.PercentOutput, 0.62);

					m_spitTimer.stop();
					m_spitTimer.reset();
					m_spitTimer.start();

					m_spitCase++;
					break;
				case 1:
					if (m_spitTimer.get() > 0.2)
					{
						m_spitCase++;
					}
					break;
				case 2:
					m_intakeVictor.set(ControlMode.PercentOutput, spitIntakeSpeed);

					if (spitIntakeSpeed < 0.2)
					{
						m_spitCase++;
					} else
					{
						spitIntakeSpeed = spitIntakeSpeed * 0.8;
					}

					break;
				}
			} else
			{
				m_intakeVictor.setInverted(true);
				m_intakeVictor.set(ControlMode.PercentOutput, 0.62);
			}

			SmartDashboard.putNumber("spitIntakeSpeed", spitIntakeSpeed);

		} else
		{
			if ((prevIntakeSpeed < -0.7) && (prevIntakeSpeed > -1.1) && currentIntakeSpeed == 0.0)
			{
				cubicShuffleIntake = true;
			}

			if (cubicShuffleIntake == true)
			{
				switch (m_intakeCase)
				{
				case 0:
					m_intakeVictor.setInverted(false);
					m_intakeVictor.set(ControlMode.PercentOutput, 0.5);

					m_rollingTimer.stop();
					m_rollingTimer.reset();
					m_rollingTimer.start();

					m_intakeCase++;
					break;
				case 1:
					if (m_rollingTimer.get() > 0.2)
					{
						m_intakeVictor.setInverted(true);
						m_intakeVictor.set(ControlMode.PercentOutput, -0.8);
						m_intakeCase++;
					} else
					{
						m_intakeVictor.setInverted(false);
						m_intakeVictor.set(ControlMode.PercentOutput, 0.5);
					}
					break;
				case 2:
					m_rollingTimer.stop();
					m_rollingTimer.reset();
					m_rollingTimer.start();
					m_intakeCase++;
					break;
				case 3:
					if (m_rollingTimer.get() > 0.3)
					{
						m_intakeVictor.setInverted(true);
						m_intakeVictor.set(ControlMode.PercentOutput, speed);
						m_intakeCase++;
					} else
					{
						m_intakeVictor.setInverted(true);
						m_intakeVictor.set(ControlMode.PercentOutput, -0.8);
					}
					break;
				case 4:
					m_rollingTimer.stop();
					m_rollingTimer.reset();
					m_rollingTimer.start();
					m_intakeCase++;
					break;
				case 5:
					if (m_rollingTimer.get() > 1.0)
					{
						m_intakeVictor.setInverted(true);
						m_intakeVictor.set(ControlMode.PercentOutput, speed);
						m_intakeCase++;
					} else
					{
						m_intakeVictor.setInverted(true);
						m_intakeVictor.set(ControlMode.PercentOutput, -0.4);
					}
					break;
				case 6:
					cubicShuffleIntake = false;
					m_intakeCase++;
					break;
				}
			} else
			{
				m_intakeVictor.setInverted(true);
				m_intakeVictor.set(ControlMode.PercentOutput, speed);

				m_intakeCase = 0;

			}
			SmartDashboard.putNumber("Intake Case", m_intakeCase);
			SmartDashboard.putNumber("Intake Timer", m_rollingTimer.get());
			SmartDashboard.putBoolean("Intake Go", cubicShuffleIntake);
			m_spitCase = 0;
		}

	}

	public void UpdateIntakePID()
	{
		if (wristDesiredPosition == 0)
		{
			m_wrist.configMotionCruiseVelocity(400, 10); // 200
			m_wrist.configMotionAcceleration(600, 10);

			m_wrist.config_kP(0, Constants.ANGLE_INTAKE_UP_P, 10);
			m_wrist.config_kI(0, Constants.ANGLE_INTAKE_UP_I, 10);
			m_wrist.config_kD(0, Constants.ANGLE_INTAKE_UP_D, 10);
			m_wrist.config_kF(0, Constants.ANGLE_INTAKE_UP_F, 10);
		} else if (wristDesiredPosition == 2)
		{

			m_wrist.configMotionCruiseVelocity(600, 10); // 200
			m_wrist.configMotionAcceleration(600, 10);

			m_wrist.config_kP(0, Constants.ANGLE_INTAKE_DOWN_P, 10);
			m_wrist.config_kI(0, Constants.ANGLE_INTAKE_DOWN_I, 10);
			m_wrist.config_kD(0, Constants.ANGLE_INTAKE_DOWN_D, 10);
			m_wrist.config_kF(0, Constants.ANGLE_INTAKE_DOWN_F, 10);

		} else
		{
			m_wrist.configMotionCruiseVelocity(400, 10); // 200
			m_wrist.configMotionAcceleration(600, 10);

			m_wrist.config_kP(0, Constants.ANGLE_INTAKE_MID_P, 10);
			m_wrist.config_kI(0, Constants.ANGLE_INTAKE_MID_I, 10);
			m_wrist.config_kD(0, Constants.ANGLE_INTAKE_MID_D, 10);
			m_wrist.config_kF(0, Constants.ANGLE_INTAKE_MID_F, 10);

		}

	}

	public double GetIntakeDesiredSpeed()
	{
		return IntakeDesiredSpeed;
	}

	public boolean GetCubicShuffleActive()
	{
		return cubicShuffleIntake;
	}

	// Wrist Motor

	public void WristManual(double wristspeed)
	{
		m_wrist.set(ControlMode.PercentOutput, wristspeed);
	}

	public enum WristPosition
	{
		Up(0), Middle(1), Down(2), Resting(3), Back(4), Ten(5), autonMiddle(6), autonBack(7), autonUp(8), autonDown(
				9), autonTen(10), Spit(11), High(12), HighHigh(13);
		public final int value;

		WristPosition(int v)
		{
			value = v;
		}
	};

	public void magicalWrist(WristPosition position)
	{

		wristDesiredPosition = position.value;
		switch (position)
		{
		case Up:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_UP / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case Middle:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_MID / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case Down:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_DOWN / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case Resting:
			m_wrist.set(ControlMode.PercentOutput, 0.0);
			break;
		case Back:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_BACK / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case Ten:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_TEN / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case autonMiddle:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_AUTONMID / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case autonBack:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_AUTONBACK / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case autonUp:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_AUTONUP / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case autonDown:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_AUTONDOWN / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case autonTen:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_AUTONTEN / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case Spit:
			WristCommandedPosition = -Constants.ANGLE_INTAKE_SPIT / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case High:
			WristCommandedPosition = -Constants.ANGLE_HIGH / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		case HighHigh:
			WristCommandedPosition = -Constants.ANGLE_HIGH_HIGH / Constants.INTAKE_UNITS;
			UpdateIntakePID();
			m_wrist.set(ControlMode.MotionMagic, WristCommandedPosition);
			break;
		default:
			m_wrist.set(ControlMode.PercentOutput, 0.0);

		}// goes with switch

		// if(MagicLimit() == true){
		// m_lead.Set(ControlMode.PercentOutput, 0.0);
		// }

	}

	public double WristCurrent()
	{
		return OutputCurrent;
	}

	public void ZeroWrist()
	{
		m_wrist.setSelectedSensorPosition(0, 0, 0);
	}

	public double GetWristDesiredPosition()
	{
		return DesiredWristPosition;
	}

	public double GetWristActualPosition()
	{
		return WristCurrentPosition; // convert
	}

	public double GetWristError()
	{
		return ClosedLoopError;
	}

	public void FastShoot(boolean on)
	{
		fastShoot = on;
	}

	public boolean GetShoot()
	{
		return fastShoot;
	}

	public double GetWristPercentOutput()
	{
		return MotorOutputWrist;
	}

	public double GetIntakeMasterPercentOutput()
	{
		return MotorOutputMaster;
	}

	public double GetIntakeSlavePercentOutput()
	{
		return MotorOutputSlave;
	}

	public boolean wristAtPosition()
	{
		if (Math.abs(GetWristError()) < 15.0)
		{
			return true;
		} else
		{
			return false;
		}

	}

	public double GetWristAngle()
	{
		// -angle / intake units = encoder ticks
		return -1.0 * Constants.INTAKE_UNITS * GetWristActualPosition();
	}

	private VictorSPX m_intakeVictor;
	private VictorSPX m_slaveVictor;
	private TalonSRX m_wrist;
	private Solenoid m_claw;
	private Solenoid m_neutralClaw;
	private Timer m_rollingTimer;
	private Timer m_spitTimer;

	private int wristDesiredPosition;

	private boolean fastShoot = false;
	private boolean neutralClawOpen = false;
	private boolean ClawOpen = false;
	private boolean ButtonA = false;
	private boolean ButtonB = false;
	private boolean ButtonC = false;
	private double TargetPosition = 0;

	private double RPM_CONVERT = (600.0 / 4096.0);

	private int m_intakeCase = 0;
	private double currentIntakeSpeed = 0.0;
	private double prevIntakeSpeed = 0.0;

	private double spitIntakeSpeed = 0.0;

	private int m_spitCase = 0;

	private boolean cubicShuffleIntake = false;

}
