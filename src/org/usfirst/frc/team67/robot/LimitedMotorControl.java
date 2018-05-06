package org.usfirst.frc.team67.robot;

import org.usfirst.frc.team67.robotutils.HotJoystick;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LimitedMotorControl
{
	private double currentPosition;
	private double previousPosition;

	protected double topLimit;
	protected double bottomLimit;
	protected float highDead;
	protected float lowDead;
	protected float highCap;
	protected float lowCap;
	protected int kTimeoutMs;
	protected TalonSRX m_lead;
	protected HotJoystick joy;

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

	public boolean MagicLimit(double targetPosition)
	{
		getCurrentPosition();

		boolean returnValue = false;

		if ((currentPosition > previousPosition) && (currentPosition > topLimit) && (targetPosition > currentPosition))
		{
			returnValue = true;
		} else if ((currentPosition < previousPosition) && (currentPosition < bottomLimit)
				&& (targetPosition < currentPosition))
		{
			returnValue = true;
		} else
		{
			returnValue = false;
		}
		previousPosition = currentPosition;
		return returnValue;

	}

	public LimitedMotorControl(int leadId)
	{
		m_lead = new TalonSRX(leadId);
		m_lead.setNeutralMode(NeutralMode.Brake);
		m_lead.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

	}

	public void Init()
	{
		m_lead.setSelectedSensorPosition(0, 0, kTimeoutMs);
	}

	public double getCurrentPosition()
	{
		return m_lead.getSelectedSensorPosition(0);
	}

	public float ManualControl(double joy)
	{
		float spdCmd;
		if (joy <= highDead && joy >= lowDead)
		{
			spdCmd = 0.0f;
		} else if (joy > highDead)
		{
			if (joy >= highCap)
			{
				spdCmd = highCap;
			} else
			{
				spdCmd = (float) ((joy - highDead) * -1.25f);
			}
		} else
		{
			if (joy <= lowCap)
			{
				spdCmd = lowCap;
			} else
			{
				spdCmd = (float) ((joy + lowDead) * -1.25);
			}
		}
		return spdCmd;

	}
}
