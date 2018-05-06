package org.usfirst.frc.team67.robot.drivetrain;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

// Distance PID Wrapper
public class DistancePIDWrapper implements PIDSource, PIDOutput
{
	public DistancePIDWrapper(Drivetrain drivetrain)
	{
		m_drivetrain = drivetrain;
	}

	@Override
	public double pidGet()
	{
		return (m_drivetrain.GetYaw());
	}

	@Override
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