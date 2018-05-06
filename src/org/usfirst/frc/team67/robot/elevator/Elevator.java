package org.usfirst.frc.team67.robot.elevator;

import org.usfirst.frc.team67.robot.LimitedMotorControl;
import org.usfirst.frc.team67.robotutils.HotJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Elevator extends LimitedMotorControl
{

	public Elevator(int leadID)
	{
		super(leadID);
		// calibrations
		highDead = 0.2f;
		lowDead = -0.2f;
		highCap = 0.8f;
		lowCap = -0.8f;
		kTimeoutMs = 10;
		kPIDLoopIdx = 0;
		kSlotIdx = 0;
		topLimit = 39000;
		// internal calibrations
		bottomLimit = -1000000;

		m_operator = new HotJoystick(1);
		/*
		 * m_lead.SetStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10,
		 * 10); m_lead.SetStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic,
		 * 10, 10);
		 */

		// magicalElevatorInitializations
	}

	public enum button
	{
		auto_highScale, // w30 etop y1
		auto_midScale, auto_lowScale, // w30 elowscale b2
		auto_carry, // whatever switch is b1
		auto_pickup, // w101 ezero a1
		auto_exchange, // w90 ezero a2
		auto_slamDunk, // wneg10FIX etoplimit y2
		auto_humanPlayer, // w15 ezero x2
		auto_package, // w0 ezero x1
		highScale, // w30 etop y1
		midScale, lowScale, // w30 elowscale b2
		carry, // whatever switch is b1
		pickup, // w101 ezero a1
		exchange, // w90 ezero a2
		slamDunk, // wneg10FIX etoplimit y2
		humanPlayer, // w15 ezero x2
		pack // w0 ezero x1
	};

	public void elevatorInit()
	{
		Init();
		m_lead.setSensorPhase(true);
		m_lead.configNominalOutputForward(0, kTimeoutMs);
		m_lead.configNominalOutputForward(0, kTimeoutMs);
		m_lead.configNominalOutputForward(1, kTimeoutMs);
		m_lead.configNominalOutputReverse(-1, kTimeoutMs);

		m_lead.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		m_lead.config_kF(0, 0.2, kTimeoutMs);
		m_lead.config_kP(0, 0.2, kTimeoutMs);
		m_lead.config_kI(0, 0.0, kTimeoutMs);
		m_lead.config_kD(0, 0, kTimeoutMs);
		m_lead.configMotionCruiseVelocity(15000, kTimeoutMs);
		m_lead.configMotionAcceleration(6000, kTimeoutMs);
		m_lead.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

	}

	public double GetElevatorError()
	{
		return m_lead.getClosedLoopError(0);
	}

	public boolean elevatorAtPosition()
	{
		if (Math.abs(GetElevatorError()) < 40.0)
		{
			return true;
		} else
		{
			return false;
		}

	}

	public void manualElevator(double output)
	{
		float spdCmd; // output
		spdCmd = ManualControl(output);
		spdCmd = Limit(spdCmd);
		m_lead.set(ControlMode.PercentOutput, spdCmd);
		firstElevator = false;

	}
	// manual control elevator
	// double is joystick output

	public void magicalElevator(button _Button)
	{

		switch (_Button)
		{
		case auto_highScale: // y1
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_TOP_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_midScale:
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_MID_SCALE_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_carry: // b1
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_SWITCH_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_package: // x1
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_pickup: // a1
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_slamDunk: // y2
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_ULTRAHIGH_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_lowScale: // b2
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_LOW_SCALE_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_humanPlayer: // x2
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case auto_exchange: // a2
			desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM_AUTO;
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;

		case highScale: // y1

			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_TOP;
				firstElevator = true;
			} else
			{
				if (m_operator.getPOV() == 315 || m_operator.getPOV() == 0 || m_operator.getPOV() == 45)
				{
					desiredElevatorPosition = desiredElevatorPosition + Constants.ELEVATOR_ADJUST;
				} else if (m_operator.getPOV() == 225 || m_operator.getPOV() == 180 || m_operator.getPOV() == 135)
				{
					desiredElevatorPosition = desiredElevatorPosition - Constants.ELEVATOR_ADJUST;
				}
			}

			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case midScale:
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_MID_SCALE;
				firstElevator = true;
			} else
			{
				if (m_operator.getPOV() == 315 || m_operator.getPOV() == 0 || m_operator.getPOV() == 45)
				{
					desiredElevatorPosition = desiredElevatorPosition + Constants.ELEVATOR_ADJUST;
				} else if (m_operator.getPOV() == 225 || m_operator.getPOV() == 180 || m_operator.getPOV() == 135)
				{
					desiredElevatorPosition = desiredElevatorPosition - Constants.ELEVATOR_ADJUST;
				}
			}
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case carry: // b1
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_SWITCH;
				firstElevator = true;
			}

			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case pack: // x1
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM;
				firstElevator = true;
			}
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case pickup: // a1
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM;
				firstElevator = true;
			}

			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case slamDunk: // y2
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_ULTRAHIGH;
				firstElevator = true;
			} else
			{
				if (m_operator.getPOV() == 315 || m_operator.getPOV() == 0 || m_operator.getPOV() == 45)
				{
					desiredElevatorPosition = desiredElevatorPosition + Constants.ELEVATOR_ADJUST;
				} else if (m_operator.getPOV() == 225 || m_operator.getPOV() == 180 || m_operator.getPOV() == 135)
				{
					desiredElevatorPosition = desiredElevatorPosition - Constants.ELEVATOR_ADJUST;
				}
			}
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case lowScale: // b2
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_LOW_SCALE;
				firstElevator = true;
			} else
			{
				if (m_operator.getPOV() == 315 || m_operator.getPOV() == 0 || m_operator.getPOV() == 45)
				{
					desiredElevatorPosition = desiredElevatorPosition + Constants.ELEVATOR_ADJUST;
				} else if (m_operator.getPOV() == 225 || m_operator.getPOV() == 180 || m_operator.getPOV() == 135)
				{
					desiredElevatorPosition = desiredElevatorPosition - Constants.ELEVATOR_ADJUST;
				}
			}
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case humanPlayer: // x2
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM;
				firstElevator = true;
			}
			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		case exchange: // a2
			if (firstElevator == false)
			{
				desiredElevatorPosition = Constants.ELEVATOR_POSITION_BOTTOM;
				firstElevator = true;
			}

			m_lead.set(ControlMode.MotionMagic, desiredElevatorPosition);
			break;
		default:
			firstElevator = false;
			m_lead.set(ControlMode.PercentOutput, 0.0);

		}// goes with switch

		if (MagicLimit(desiredElevatorPosition) == true)
		{
			m_lead.set(ControlMode.PercentOutput, 0.0);
		}

	}

	// any position to variable position x
	// input is position X, which is a percentage of encoder ticks where 33,000 (out
	// of 38,000) is 1.0 and 800 (out of 0) is 0.0
	// named "magicalElevator" because it is an elevator function that uses motion
	// magic, of course
	public double elevatorCurrent()
	{
		return m_lead.getOutputCurrent();
	}

	public double elevatorCommanded()
	{
		return m_lead.getMotorOutputPercent();
	}

	public void resetElevator()
	{
		m_lead.setSelectedSensorPosition(0, 0, 0);
	}

	private HotJoystick m_operator;
	// magicalElevator
	// basics
	private int kSlotIdx; // cali??????????????
	private int kPIDLoopIdx; // cali???????????

	private double desiredElevatorPosition;
	private boolean firstElevator;

}
