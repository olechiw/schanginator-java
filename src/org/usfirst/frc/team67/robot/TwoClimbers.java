package org.usfirst.frc.team67.robot;

import org.usfirst.frc.team67.robotutils.HotJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TwoClimbers
{
	public static final int TALON_CLIMBER_R = 10;
	public static final int TALON_CLIMBER_L = 15;

	public TwoClimbers()
	{
		m_climberMotor = new TalonSRX(TALON_CLIMBER_R);
		m_climberSlave = new TalonSRX(TALON_CLIMBER_L);

		// calibrations
		highDeadband = 0.2f;
		lowDeadband = -0.2f;
		highCap = 0.8f;
		lowCap = -0.8f;
		kTimeoutMs = 0;
		kPIDLoopIdx = 0;
		kSlotIdx = 0;
		topLimit = 470514.8717955f; // 17.5 inch //cali 120989.538462 is 17.5 inches
		bottomLimit = -5000000f; // cali
		maxOutput = 5.0f;

		positionTop = 1;
		positionBottom = 1;
		positionNinety = 1;

	}

	public enum button
	{
		aPressed, // =0
		bPressed, // =1
		yPressed // =2
	};

	public boolean climberReset()
	{
		if (m_climberMotor.getOutputCurrent() > maxOutput)
		{
			m_climberMotor.setSelectedSensorPosition(0, 0, 10);
			m_climberMotor.set(ControlMode.PercentOutput, 0.0);
			return true;
		} else
		{
			m_climberMotor.set(ControlMode.PercentOutput, -0.05);
			return false;
		}

	}

	public void zeroClimber()
	{
		m_climberMotor.setSelectedSensorPosition(0, 0, 0);
	}

	public void climberInit()
	{
		m_climberMotor.setSelectedSensorPosition(0, 0, 10);
		m_climberMotor.setSensorPhase(true);
		m_climberMotor.configNominalOutputForward(0, kTimeoutMs);
		m_climberMotor.configNominalOutputReverse(0, kTimeoutMs);
		m_climberMotor.configPeakOutputForward(1, kTimeoutMs);
		m_climberMotor.configPeakOutputReverse(-1, kTimeoutMs);

		m_climberMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		m_climberMotor.config_kF(0, 0.2, kTimeoutMs);
		m_climberMotor.config_kP(0, 0.2, kTimeoutMs);
		m_climberMotor.config_kI(0, 0.0, kTimeoutMs);
		m_climberMotor.config_kD(0, 0, kTimeoutMs);
		m_climberMotor.configMotionCruiseVelocity(15000, kTimeoutMs);
		m_climberMotor.configMotionAcceleration(6000, kTimeoutMs);
		m_climberMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		m_climberSlave.follow(m_climberMotor);

	}

	public void manualClimber(float joy)
	{

		float spdCmd; // output
		// spdCmd = ManualControl(joy); //set speed command to manual control
		spdCmd = Limit(joy); // add limit to speed command
		m_climberMotor.set(ControlMode.PercentOutput, spdCmd); // implement speed command

	}

	public void magicalClimber(TwoClimbers.button _Button)
	{
		switch (_Button)
		{
		case yPressed:
			m_climberMotor.set(ControlMode.MotionMagic, positionTop);
			break;
		case bPressed:
			m_climberMotor.set(ControlMode.MotionMagic, positionNinety);
			break;
		case aPressed:
			m_climberMotor.set(ControlMode.MotionMagic, positionBottom);
			break;

		}// goes with switch

		if (Limit((float) m_climberMotor.getMotorOutputPercent()) == 0)
		{
			m_climberMotor.set(ControlMode.PercentOutput, 0.0);
		} // goes with if

	}

	public double getClimberMasterCurrent()
	{
		return m_climberMotor.getOutputCurrent();
	}

	public double getClimberSlaveCurrent()
	{
		return m_climberSlave.getOutputCurrent();
	}

	public double climberCommanded()
	{
		return m_climberMotor.getMotorOutputPercent();
	}

	public float Limit(float spdCmd)
	{
		getCurrentPosition();
		if (currentPosition >= topLimit && spdCmd > 0.0)
		{
			spdCmd = 0;
		} else if (currentPosition <= bottomLimit && spdCmd < 0.0)
		{
			spdCmd = 0;
		}
		return spdCmd;

	}

	public double getCurrentPosition()
	{
		double newPosition;
		newPosition = m_climberMotor.getSelectedSensorPosition(0);
		if (m_climberMotor.getLastError().value == 0)
		{
			currentPosition = newPosition;
		}

		return currentPosition;

	}

	private TalonSRX m_climberMotor;
	private TalonSRX m_climberSlave;
	private HotJoystick m_hotJoy;
	private double climberPosition = 0.0; // output of getGurrentPosition
	private float highDeadband; // cali
	private float lowDeadband; // cali
	private float highCap; // cali
	private float lowCap; // cali
	private int climberID; // cali
	private int climberFollowerID;
	private float maxOutput;

	private float topLimit;
	private float bottomLimit;

	private double currentPosition;

	private float positionTop;
	private float positionBottom;
	private float positionNinety;

	// magicalElevator
	// basics
	private int kSlotIdx; // cali??????????????
	private int kPIDLoopIdx; // cali???????????
	private int kTimeoutMs; // cali

}
