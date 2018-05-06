package org.usfirst.frc.team67.robot.drivetrain.pid;

import org.usfirst.frc.team67.robot.drivetrain.Constants;
import org.usfirst.frc.team67.robot.drivetrain.Drivetrain;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AnglePIDWrapper implements PIDSource, PIDOutput
{
	public AnglePIDWrapper(Drivetrain drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	public double pidGet()
	{
		return (m_drivetrain.GetYaw());
	}

	public void pidWrite(double output)
	{
		// SmartDashboard::PutNumber("Turn PID Output", output);
		if ((-output * m_drivetrain.turnPIDSpeed < 0.15) && (-output * m_drivetrain.turnPIDSpeed > 0)
				&& (Math.abs(m_drivetrain.GetAnglePIDError()) < 4))
		{
			m_drivetrain.SetTurn(Constants.MIN_TURN_SPEED);
		} else if ((-output * m_drivetrain.turnPIDSpeed > -0.15) && (-output * m_drivetrain.turnPIDSpeed < 0)
				&& (Math.abs(m_drivetrain.GetAnglePIDError()) < 4))
		{
			m_drivetrain.SetTurn(-Constants.MIN_TURN_SPEED);
		} else
		{
			m_drivetrain.SetTurn(-output * m_drivetrain.turnPIDSpeed);
		}

		m_drivetrain.anglePIDOutput = output;

	}

	private Drivetrain m_drivetrain;

	@Override
	public void setPIDSourceType(PIDSourceType pidSource)
	{
		// TODO Auto-generated method stub

	}

	@Override
	public PIDSourceType getPIDSourceType()
	{
		// TODO Auto-generated method stub
		return null;
	}
};