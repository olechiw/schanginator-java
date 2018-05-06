package org.usfirst.frc.team67.robot;

import org.usfirst.frc.team67.robotutils.HotJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/*
 *   Revolutions   1          Input   18      Inches       13/64
 *                       x                 X
 *   counts   4096            Output  24      Revolutions    1
 *
 *
 */

//or
//88.6/x = 7/1 x/newt = 18/24   Final Answer: 16.8791208791

//belt ratio 18:24, geared slower, gearbox ratio 7:1
//13/64th of inch per turn. moves exactly 18 inches. =88.6153846154 revolutions to move exactly 18 inches
//1024 counts per revolution
// = encoder counts for top limit and bottom limit

public class Climber extends LimitedMotorControl
{
	public static final int TALON_CLIMBER_R = 10;

	public Climber(int leadID)
	{
		super(leadID);
		// calibrations
		highDeadband = 0.2f;
		lowDeadband = -0.2f;
		highCap = 0.8f;
		lowCap = -0.8f;
		kTimeoutMs = 0;
		kPIDLoopIdx = 0;
		kSlotIdx = 0;
		topLimit = 470514.8717955; // 17.5 inch //cali 120989.538462 is 17.5 inches
		bottomLimit = -200000; // cali
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
		if (m_lead.getOutputCurrent() > maxOutput)
		{
			Init();
			m_lead.set(ControlMode.PercentOutput, 0.0);
			return true;
		} else
		{
			m_lead.set(ControlMode.PercentOutput, -0.05);
			return false;
		}
	}

	public void zeroClimber()
	{
		m_lead.setSelectedSensorPosition(0, 0, 0);
	}

	public void climberInit()
	{
		Init();
		m_lead.setSensorPhase(true);
		m_lead.configNominalOutputForward(0, kTimeoutMs);
		m_lead.configNominalOutputReverse(0, kTimeoutMs);
		m_lead.configPeakOutputForward(1, kTimeoutMs);
		m_lead.configPeakOutputReverse(-1, kTimeoutMs);

		m_lead.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		m_lead.config_kF(0, 0.2, kTimeoutMs);
		m_lead.config_kP(0, 0.2, kTimeoutMs);
		m_lead.config_kI(0, 0.0, kTimeoutMs);
		m_lead.config_kD(0, 0, kTimeoutMs);
		m_lead.configMotionCruiseVelocity(15000, kTimeoutMs);
		m_lead.configMotionAcceleration(6000, kTimeoutMs);
		m_lead.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

	}

	public void manualClimber(float joy)
	{
		float spdCmd; // output
		// spdCmd = ManualControl(joy); //set speed command to manual control
		spdCmd = Limit(joy); // add limit to speed command
		m_lead.set(ControlMode.PercentOutput, spdCmd); // implement speed command

	}

	public void magicalClimber(button _Button)
	{
		switch (_Button)
		{
		case yPressed:
			m_lead.set(ControlMode.MotionMagic, positionTop);
			break;
		case bPressed:
			m_lead.set(ControlMode.MotionMagic, positionNinety);
			break;
		case aPressed:
			m_lead.set(ControlMode.MotionMagic, positionBottom);
			break;

		}// goes with switch

		if (Limit((float) m_lead.getMotorOutputPercent()) == 0)
		{
			m_lead.set(ControlMode.PercentOutput, 0.0);
		} // goes with if

	}

	public double climberCommanded()
	{
		return m_lead.getMotorOutputPercent();
	}

	private TalonSRX m_climberMotor;
	private HotJoystick m_hotJoy;

	private double climberPosition; // output of getGurrentPosition
	private float highDeadband; // cali
	private float lowDeadband; // cali
	private float highCap; // cali
	private float lowCap; // cali
	private int climberID; // cali
	private int climberFollowerID;
	private float maxOutput;

	private float positionTop;
	private float positionBottom;
	private float positionNinety;

	// magicalElevator
	// basics
	private int kSlotIdx; // cali??????????????
	private int kPIDLoopIdx; // cali???????????
	private int kTimeoutMs; // cali

}
