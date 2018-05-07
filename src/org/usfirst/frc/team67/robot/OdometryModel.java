package org.usfirst.frc.team67.robot;
import org.usfirst.frc.team67.robot.drivetrain.Drivetrain;

public class OdometryModel
{
	public static final float ROBOT_LENGTH = 13.25f;
	public static final float RADIANS_CONVERSION = 59.2958f;

	private double displacement;
	private double localizationX;
	private double localizationY;
	private double angle;

	public OdometryModel()
	{
		displacement = 0;
		angle = 0;

	}

	public double GetDriveEncoderX()
	{
		localizationX = (GetDriveDisplacementCenter() * Math.cos(GetDriveEncoderAngle() / 2.0));
		return localizationX;
	}

	public double GetDriveEncoderY()
	{
		localizationY = (GetDriveDisplacementCenter() * Math.sin(GetDriveEncoderAngle() / 2.0));
		return localizationY;
	}

	public double GetDriveEncoderAngle()
	{
		angle = ((m_drivetrain.GetRightDistance() - m_drivetrain.GetLeftDistance()) / (2 * ROBOT_LENGTH)
				* RADIANS_CONVERSION);
		return angle;

	}

	public double GetDriveDisplacementCenter()
	{
		displacement = ((m_drivetrain.GetRightDistance() + m_drivetrain.GetLeftDistance()) / 2.0);
		return displacement;

	}

	public Drivetrain m_drivetrain;

}
