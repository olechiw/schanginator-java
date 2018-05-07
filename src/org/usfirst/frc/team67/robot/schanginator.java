package org.usfirst.frc.team67.robot;

import java.math.BigInteger;

import org.usfirst.frc.team67.robot.drivetrain.Drivetrain;
import org.usfirst.frc.team67.robot.elevator.Elevator;
import org.usfirst.frc.team67.robot.intake.Intake;
import org.usfirst.frc.team67.robotutils.HotJoystick;
import org.usfirst.frc.team67.robotutils.JoystickMapping;
import org.usfirst.frc.team67.robotutils.Point2D;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class schanginator extends TimedRobot
{
	public static final int I2C_ADDRESS = 0x1E;

	public static final int I2C_COMMAND = 0x03;
	public static final int I2C_READ = 0x04;

	public static final int I2C_MODE_ACTIVE = 0x00;
	public static final int I2C_FREQUENCY_60HZ = 0x36;

	public static final int I2C_COMMAND_CAL_BLACK = 0x42;
	public static final int I2C_COMMAND_CAL_WHITE = 0x43;

	public static final double driverDeadBandHigh = .1;
	public static final double driverDeadBandLow = -.1;

	private HotJoystick m_driver;
	private HotJoystick m_operator;
	private JoystickMapping l_mapJoyStick;
	private JoystickMapping r_mapJoyStick;

	// PowerDistributionPanel m_pdp;

	private Drivetrain m_drivetrain;
	// OdometryModel m_odometryModel;
	private Intake m_intake;
	private Elevator m_elevator;
	private TwoClimbers m_climber;

	private Timer m_autonTimer;
	private Timer m_servoTimer;
	private Timer m_autonSpitTimer;
	private Timer m_autonRollinTimer;
	private Timer m_rollingTimer;
	private Timer m_autonZeroWristTimer;
	private Timer m_wristAngleTimer;

	private boolean servoRun = false;
	private int servoCase = 0;

	private boolean resetHasRun = false;

	private boolean climberSafe = false;
	private int climberCase = 0;

	private int m_autonCase = 0;

	private int m_intakeCase = 0;
	// public double currentIntakeSpeed = 0.0;
	// public double prevIntakeSpeed = 0.0;

	public double m_autonSwitchDriveToTurnDistance = 0;
	public double m_autonSwitchTurnAngle = 0;
	public double m_autonSwitchDistance = 0;
	public double m_autonSwitchForwardDistance = 0;

	public double m_autonScaleDistance;
	public double m_autonScaleForwardDistance;
	public double m_autonScaleBackUpDistance;
	public double m_autonScaleBonusCubeAngle;
	public double m_autonScaleCrossFieldDistance;
	public double m_autonScaleTurnAngle2;
	public double m_autonScaleBonusCubeDistance;
	public double m_autonScaleTurnAngle;
	public double m_autonScaleAwayWithCubeDistance;

	public double m_autonScaleBonusCubeBackupDistance;
	public double m_autonScaleBonusCubeBackupAngle;

	public double m_autonScaleSmallBackUpDistance;

	public double m_autonScaleBonusCubeForwardDistance;
	public double m_autonScaleSmallGoForwardDistance;

	public double m_autonSwitchCTFDistance;
	public double m_autonSwitchCTFAngle;
	public double m_autonSwitchBonusCubeBackupDistance;
	public double m_autonSwitchBonusCubeAngle;
	public double m_autonSwitchBonusCubeDistance;
	public double m_autonSwitchForwardDistance2;

	public double m_autonSwitchBackUpDistance;
	public double m_autonSwitchSmallBackUpDistance;
	public double m_autonSwitchTurnToScale1;
	public double m_autonSwitchDriveToScale1;
	public double m_autonSwitchTurnToScale2;
	public double m_autonSwitchDriveToScale2;

	public double m_autonScaleBonusCubePickupAngle;
	public double m_autonScaleBonusCubePickupDistance;

	public double m_autonScaleCompatibleDistance1;
	public double m_autonScaleCompatibleAngle1;
	public double m_autonScaleCompatibleDistance2;
	public double m_autonScaleCompatibleAngle2;
	public double m_autonScaleCompatibleBonusAngle;

	boolean allianceSwitch = false;
	boolean allianceScale = false;
	boolean switchScaleAuton = false;

	boolean shifted = false;
	boolean shiftedOld;

	public double autonPosition = 2;
	public double matchTime;

	public double autonPlanOne; // this determines the path that will be taken during autonomous.
	public double autonPlanTwo;
	public double autonPlanThree;
	public double autonPlanFour;
	public double autonPlan = 0.0;

	int FieldConfig;/*
					 * Schang mods for anti-tip start
					 */
	public double antiTipHigh = 25500.0;/*
										 * Elevator threshold to enable anti-tip.
										 */
	public double driverLY_Final = 0.0;/*
										 * LY variable send to Arcade Drive during TeleOp
										 */
	public double antiTipSclFx = 0.25;/*
										 * Note: antiTipSclFx is a number between 0.0 and 1.0.antiTipSclFx=1.0 will
										 * eliminate all filtering antiTipSclFx set low will filter/ smooth quick
										 * changes to LY input// Schang mods for anti-tip end
										 */

	public double tsf; // turn scale factor
	public double dsf; // drive scale factor

	// Color Sensor
	I2C m_colorSensor; // I2C Color Sensor
	byte[] buffer; // Buffer to hold color number

	private final Point2D[] LeftJoyStickMap = new Point2D[]
	{ new Point2D(-1, -1), new Point2D(-.9, -.7), new Point2D(-.6, -.3), new Point2D(-.1, 0), new Point2D(.1, 0),
			new Point2D(.6, .3), new Point2D(.9, .7), new Point2D(1, 1) };

	private final Point2D[] RightJoyStickMap =
	{ new Point2D(-1, -0.8), new Point2D(-.9, -.5), new Point2D(-.6, -.2), new Point2D(-.1, 0), new Point2D(.1, 0),
			new Point2D(.6, .2), new Point2D(.9, .5), new Point2D(1, 0.8) };

	schanginator()
	/* : m_pdp(0) */ {
		m_driver = new HotJoystick(0);
		l_mapJoyStick = new JoystickMapping(LeftJoyStickMap, driverDeadBandLow, driverDeadBandHigh);
		r_mapJoyStick = new JoystickMapping(RightJoyStickMap, driverDeadBandLow, driverDeadBandHigh);
		m_operator = new HotJoystick(1); // normally 1 FIXME
		m_elevator = new Elevator(org.usfirst.frc.team67.robot.elevator.Constants.TALON_ELEVATOR_L);
		m_climber = new TwoClimbers();
		m_intake = new Intake();
		m_autonTimer = new Timer();
		m_servoTimer = new Timer();
		m_autonSpitTimer = new Timer();
		m_autonRollinTimer = new Timer();
		m_rollingTimer = new Timer();
		m_autonZeroWristTimer = new Timer();

		// Color Sensor
		m_colorSensor = new I2C(I2C.Port.kOnboard, I2C_ADDRESS);
		// buffer = (uint8_t) - 1;
	}

	void RobotInit()
	{
		m_elevator.elevatorInit();
		m_climber.climberInit();
		m_intake.ZeroWrist();

	}

	void DisabledPeriodic()
	{
		autonPlanOne = SmartDashboard.getNumber("Auto Type", 0.0);
		SmartDashboard.putNumber("Auton Type", autonPlanOne);

		autonPlanTwo = SmartDashboard.getNumber("Auto Type 2", 0.0);
		SmartDashboard.putNumber("Auton Type 2", autonPlanTwo);

		autonPlanThree = SmartDashboard.getNumber("Auto Type 3", 0.0);
		SmartDashboard.putNumber("Auton Type 3", autonPlanThree);

		autonPlanFour = SmartDashboard.getNumber("Auto Type 4", 0.0);
		SmartDashboard.putNumber("Auton Type 4", autonPlanFour);

		autonPosition = SmartDashboard.getNumber("Auto Position", 0.0);
		SmartDashboard.putNumber("Auton Position", autonPosition);

		SmartDashboard.putNumber("Elevator: currentPosition", m_elevator.getCurrentPosition());
		SmartDashboard.putNumber("Climber : currentPosition", m_climber.getCurrentPosition());
		m_autonCase = 0;
	}

	void AutonomousInit()
	{
		m_drivetrain.ZeroDistance();
		m_drivetrain.ZeroGyro();
		m_drivetrain.ResetAnglePID();
		m_drivetrain.ResetDistancePID();
		m_elevator.elevatorInit();
		m_climber.climberInit();
		m_intake.ZeroWrist();

		m_autonCase = 0;

		FieldConfig = FieldConfiguration();

		autonPlanOne = SmartDashboard.getNumber("Auto Type", 0.0);
		autonPlanTwo = SmartDashboard.getNumber("Auto Type 2", 0.0);
		autonPlanThree = SmartDashboard.getNumber("Auto Type 3", 0.0);
		autonPlanFour = SmartDashboard.getNumber("Auto Type 4", 0.0);
		autonPosition = SmartDashboard.getNumber("Auto Position", 0.0);

		if (autonPosition == 0)
		{ // left start
			if (FieldConfig == 1)
			{ // Same side scale, same side switch
				if (autonPlanOne == 1)
				{ // 2X Scale
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -67.0;
					m_autonScaleSmallBackUpDistance = -10;
					m_autonScaleBonusCubeForwardDistance = 10.0;

				} else if (autonPlanThree == 5)
				{ // 2x compatible
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -90.0;
					m_autonScaleSmallBackUpDistance = -1.5;
					m_autonScaleBonusCubeForwardDistance = 10.0;
					m_autonScaleCompatibleDistance1 = -90.0;
					m_autonScaleCompatibleBonusAngle = -151.0;
					m_autonScaleCompatibleAngle1 = 0.0;
					m_autonScaleCompatibleDistance2 = 40.0;
					m_autonScaleCompatibleAngle2 = -90.0;

				} else if (autonPlanOne == 2)
				{ // Same Side Switch
					m_autonSwitchTurnAngle = -88.0;
					m_autonSwitchForwardDistance = 150.0;
					m_autonSwitchDistance = 19.0;
					m_autonSwitchCTFDistance = 60.0;
					m_autonSwitchCTFAngle = -90.0;
					m_autonSwitchBonusCubeBackupDistance = -4.0;
					m_autonSwitchBonusCubeAngle = -135.0;
					m_autonSwitchBonusCubeDistance = 8.0;
					m_autonSwitchForwardDistance2 = 50.0;

				} else if (autonPlanOne == 4)
				{ // Same Side Scale and Switch
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 255.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -148.0;
					m_autonScaleBonusCubeDistance = 53.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -67.0;
					m_autonScaleSmallBackUpDistance = -2.5;
					m_autonScaleSmallGoForwardDistance = 8;
					m_autonScaleBonusCubeForwardDistance = 17.0;

				} else
				{
					// do nothing
				}
			} else if (FieldConfig == 2)
			{ // Opposite Opposite Field Configuration
				if (autonPlanTwo == 3)
				{ // Cross the field scale
					m_autonScaleTurnAngle = -90.0; // -67.0
					m_autonScaleForwardDistance = 225.0;
					m_autonScaleDistance = 33.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 120.0;
					m_autonScaleCrossFieldDistance = 233.0;
					m_autonScaleTurnAngle2 = 30.0;

				} else
				{
					// do nothing
				}

			} else if (FieldConfig == 3)
			{ // Same side scale, opposite side switch
				if (autonPlanThree == 1)
				{ // 2x Same Side Scale
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -67.0;
					m_autonScaleSmallBackUpDistance = -10;
					m_autonScaleBonusCubeForwardDistance = 10.0;

				} else if (autonPlanThree == 5)
				{ // 2x compatible
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -90.0;
					m_autonScaleSmallBackUpDistance = -1.5;
					m_autonScaleBonusCubeForwardDistance = 10.0;
					m_autonScaleCompatibleDistance1 = -90.0;
					m_autonScaleCompatibleBonusAngle = -151.0;
					m_autonScaleCompatibleAngle1 = 0.0;
					m_autonScaleCompatibleDistance2 = 40.0;
					m_autonScaleCompatibleAngle2 = -90.0;

				} else if (autonPlanThree == 6)
				{ // Same Side Scale and Opposite Side Switch TODO
					m_autonScaleTurnAngle = -52.0;
					m_autonScaleForwardDistance = 255.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -148.0;
					m_autonScaleBonusCubeDistance = 53.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = -67.0;
					m_autonScaleSmallBackUpDistance = -10;
					m_autonScaleBonusCubeForwardDistance = 10.0;
					m_autonScaleBonusCubePickupAngle = 0;
					m_autonScaleBonusCubePickupDistance = 0;
				} else
				{
					// do nothing
				}
			} else if (FieldConfig == 4)
			{ // Opposite side scale, same side switch
				if (autonPlanFour == 2)
				{ // Same side switch
					m_autonSwitchTurnAngle = -88.0;
					m_autonSwitchForwardDistance = 150.0;
					m_autonSwitchDistance = 19.0;
					m_autonSwitchCTFDistance = 60.0;
					m_autonSwitchCTFAngle = -90.0;
					m_autonSwitchBonusCubeBackupDistance = -4.0;
					m_autonSwitchBonusCubeAngle = -135.0;
					m_autonSwitchBonusCubeDistance = 8.0;
					m_autonSwitchForwardDistance2 = 50.0;

				} else if (autonPlanFour == 3)
				{ // Cross the field
					m_autonScaleTurnAngle = -90.0; // -67.0
					m_autonScaleForwardDistance = 225.0;
					m_autonScaleDistance = 33.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 120.0;
					m_autonScaleCrossFieldDistance = 233.0;
					m_autonScaleTurnAngle2 = 30.0;
				} else
				{
					// do nothing
				}
			} else
			{ // this should never happen
				// do nothing
			}

		} else if (autonPosition == 1)
		{ // center start
			if (FieldConfig == 1)
			{ // left left
				m_autonSwitchTurnAngle = 45.0;
				m_autonSwitchDistance = 96.0;
				m_autonSwitchForwardDistance = 22.0;
				m_autonSwitchBackUpDistance = -50.0;
				m_autonSwitchSmallBackUpDistance = -10.0; // -6.7;
				m_autonSwitchBonusCubeAngle = -50.0;
				m_autonSwitchBonusCubeDistance = 30.0;
				m_autonSwitchTurnToScale1 = 0.0; // 35.0; <- use this for stop-value
				m_autonSwitchDriveToScale1 = 40.0;
				m_autonSwitchTurnToScale2 = 0.0;
				m_autonSwitchDriveToScale2 = 162.0;
			} else if (FieldConfig == 2)
			{ // right right
				m_autonSwitchTurnAngle = -45.0;
				m_autonSwitchDistance = 71.5;
				m_autonSwitchForwardDistance = 41.0;
				m_autonSwitchBackUpDistance = -50.0;
				m_autonSwitchSmallBackUpDistance = -10.0;
				m_autonSwitchBonusCubeAngle = 50.0;
				m_autonSwitchBonusCubeDistance = 30.0;
				m_autonSwitchTurnToScale1 = 0.0; // -35.0; <- use this for stop-value
				m_autonSwitchDriveToScale1 = 40.0;
				m_autonSwitchTurnToScale2 = 0.0;
				m_autonSwitchDriveToScale2 = 162.0;
			} else if (FieldConfig == 3)
			{ // right left
				m_autonSwitchTurnAngle = -45.0;
				m_autonSwitchDistance = 71.5;
				m_autonSwitchForwardDistance = 41.0;
				m_autonSwitchBackUpDistance = -50.0;
				m_autonSwitchSmallBackUpDistance = -10.0;
				m_autonSwitchBonusCubeAngle = 50.0;
				m_autonSwitchBonusCubeDistance = 30.0;
				m_autonSwitchTurnToScale1 = 0.0; // 90.0; <- use this for stop-value
				m_autonSwitchDriveToScale1 = 40.0;
				m_autonSwitchTurnToScale2 = 0.0;
				m_autonSwitchDriveToScale2 = 162.0;
			} else if (FieldConfig == 4)
			{ // left right
				m_autonSwitchTurnAngle = 45.0;
				m_autonSwitchDistance = 96.0;
				m_autonSwitchForwardDistance = 22.0;
				m_autonSwitchBackUpDistance = -50.0;
				m_autonSwitchSmallBackUpDistance = -10.0;
				m_autonSwitchBonusCubeAngle = -50.0;
				m_autonSwitchBonusCubeDistance = 30.0;
				m_autonSwitchTurnToScale1 = 0.0; // 90.0; <- use this for stop-value
				m_autonSwitchDriveToScale1 = 40.0;
				m_autonSwitchTurnToScale2 = 0.0;
				m_autonSwitchDriveToScale2 = 162.0;
			}

		} else if (autonPosition == 2)
		{ // right start
			if (FieldConfig == 1)
			{ // Same Side Switch & Same Side Scale
				if (autonPlanOne == 1)
				{ // 2x Scale - Same Side
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 67.0;
					m_autonScaleSmallBackUpDistance = -10;
					m_autonScaleBonusCubeForwardDistance = 10.0;

				} else if (autonPlanOne == 2)
				{ // Same Side Switch
					m_autonSwitchTurnAngle = 90.0;
					m_autonSwitchForwardDistance = 150.0;
					m_autonSwitchDistance = 15.0;
					m_autonSwitchCTFDistance = 60.0;
					m_autonSwitchCTFAngle = -90.0;
					m_autonSwitchBonusCubeBackupDistance = -4.0;
					m_autonSwitchBonusCubeAngle = -135.0;
					m_autonSwitchBonusCubeDistance = 8.0;
					m_autonSwitchForwardDistance2 = 50.0;

				} else if (autonPlanOne == 4)
				{ // Same Side Switch and Scale
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 255.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 148.0;
					m_autonScaleBonusCubeDistance = 53.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 67.0;
					m_autonScaleSmallBackUpDistance = -3.0;
					m_autonScaleSmallGoForwardDistance = 8;
					m_autonScaleBonusCubeForwardDistance = 17.0;

				} else if (autonPlanThree == 5)
				{ // 2x compatible
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 90.0;
					m_autonScaleSmallBackUpDistance = -1.5;
					m_autonScaleBonusCubeForwardDistance = 10.0;
					m_autonScaleCompatibleDistance1 = -90.0;
					m_autonScaleCompatibleBonusAngle = 151.0;
					m_autonScaleCompatibleAngle1 = 0.0;
					m_autonScaleCompatibleDistance2 = 40.0;
					m_autonScaleCompatibleAngle2 = 90.0;
				} else
				{
					// do nothing
				}
			} else if (FieldConfig == 2)
			{ // Opposite scale Opposite switch
				if (autonPlanTwo == 3)
				{ // Cross The Field
					m_autonScaleTurnAngle = 90.0;
					m_autonScaleForwardDistance = 225.0;
					m_autonScaleDistance = 30.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -120.0;
					m_autonScaleCrossFieldDistance = 233.0;
					m_autonScaleTurnAngle2 = -30.0;

				} else
				{
					// do nothing
				}
			} else if (FieldConfig == 3)
			{ // Same side Scale, Opposite Side Switch
				if (autonPlanThree == 1)
				{ // 2 cube scale
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 67.0;
					m_autonScaleSmallBackUpDistance = -10;
					m_autonScaleBonusCubeForwardDistance = 10.0;

				} else if (autonPlanThree == 6)
				{ // Same side scale, opposite side switch TODO
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 255.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0; // increase me
					m_autonScaleBonusCubeAngle = 90.0; // 148.0;
					m_autonScaleBonusCubeDistance = 155.0; // cross the field distance
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 67.0;
					m_autonScaleSmallBackUpDistance = -1.5;
					m_autonScaleBonusCubeForwardDistance = 20.0;
					m_autonScaleBonusCubePickupAngle = 180.0;
					m_autonScaleBonusCubePickupDistance = 10.0;

				} else if (autonPlanThree == 5)
				{ // 2x compatible
					m_autonScaleTurnAngle = 52.0;
					m_autonScaleForwardDistance = 258.0;
					m_autonScaleDistance = 10.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = 142.0;
					m_autonScaleBonusCubeDistance = 56.0; // 65;
					m_autonScaleBonusCubeBackupDistance = -57.0;
					m_autonScaleBonusCubeBackupAngle = 90.0;
					m_autonScaleSmallBackUpDistance = -1.5;
					m_autonScaleBonusCubeForwardDistance = 10.0;
					m_autonScaleCompatibleDistance1 = -90.0;
					m_autonScaleCompatibleBonusAngle = 151.0;
					m_autonScaleCompatibleAngle1 = 0.0;
					m_autonScaleCompatibleDistance2 = 40.0;
					m_autonScaleCompatibleAngle2 = 90.0;

				} else
				{
					// do nothing
				}
			} else if (FieldConfig == 4)
			{ // Opposite side scale, same side switch
				if (autonPlanFour == 2)
				{ // Same side switch
					m_autonSwitchTurnAngle = 90.0;
					m_autonSwitchForwardDistance = 150.0;
					m_autonSwitchDistance = 15.0;
					m_autonSwitchCTFDistance = 60.0;
					m_autonSwitchCTFAngle = -90.0;
					m_autonSwitchBonusCubeBackupDistance = -4.0;
					m_autonSwitchBonusCubeAngle = -135.0;
					m_autonSwitchBonusCubeDistance = 8.0;
					m_autonSwitchForwardDistance2 = 50.0;

				} else if (autonPlanFour == 3)
				{ // Cross the field
					m_autonScaleTurnAngle = 90.0;
					m_autonScaleForwardDistance = 225.0;
					m_autonScaleDistance = 30.0;
					m_autonScaleBackUpDistance = -10.0;
					m_autonScaleBonusCubeAngle = -120.0;
					m_autonScaleCrossFieldDistance = 233.0;
					m_autonScaleTurnAngle2 = -30.0;
				} else
				{
					// do nothing
				}
			} else
			{ // this should never happen
				// do nothing
			}
		} else
		{ // this should never happen
			m_autonScaleTurnAngle = 0;
			m_autonScaleForwardDistance = 0;
			m_autonScaleDistance = 0;
			m_autonScaleBackUpDistance = 0;
			m_autonScaleBonusCubeAngle = 0;
		}

		AutonDashboardOut();
	} // end autonInit

	void AutonomousPeriodic()
	{
		// --------------------------------------------------------------------------------------------//
		// Auton/Match Recording
		// --------------------------------------------------------------------------------------------//
		matchTime = DriverStation.getInstance().getMatchTime();
		m_intake.ReadSensors();

		if ((autonPosition == 0) || (autonPosition == 2))
		{ // Left or Right Side Start

			if ((FieldConfig == 1 && autonPlanOne == 1) || (FieldConfig == 3 && autonPlanThree == 1))
			{ // Same Side Scale, 2 Cube
				shifted = (matchTime > 0.5);
				if (shifted == true && shiftedOld == false)
				{
					m_drivetrain.SetShift(true);
				}
				autonPlan1_2xScale();
				autonPlan = 1.0;

			} else if ((FieldConfig == 1) && (autonPlanOne == 4))
			{ // Same Side 1-Scale 1-Switch
				shifted = (matchTime > 0.5);
				if (shifted == true && shiftedOld == false)
				{
					m_drivetrain.SetShift(true);
				}
				autonPlan4_SameSideScSw();
				autonPlan = 4.0;

			} else if ((FieldConfig == 1 && autonPlanOne == 2) || (FieldConfig == 4 && autonPlanFour == 2))
			{ // same side switch
				autonPlan2_SameSideSw();
				autonPlan = 2.0;

			} else if ((FieldConfig == 2 && autonPlanTwo == 3) || (FieldConfig == 4 && autonPlanFour == 3))
			{ // cross the field scale
				shifted = (matchTime > 0.5);
				if (shifted == true && shiftedOld == false)
				{
					m_drivetrain.SetShift(true);
				}
				autonPlan3_CrossTheFieldSc();
				autonPlan = 3.0;

			} else if ((FieldConfig == 1 && autonPlanOne == 5) || (FieldConfig == 3 && autonPlanThree == 5))
			{ // 2-cube same side compatible
				shifted = (matchTime > 0.5);
				if (shifted == true && shiftedOld == false)
				{
					m_drivetrain.SetShift(true);
				}

				autonPlan5_2xScaleCompatible();
				autonPlan = 5.0;

			} else if ((FieldConfig == 3) && (autonPlanThree == 6))
			{ // Same Side Scale, Opposite Side Switch *NOT DONE*
				shifted = (matchTime > 0.5);
				if (shifted == true && shiftedOld == false)
				{
					m_drivetrain.SetShift(true);
				}
				// autonPlan6_SameSideScOppSideSw(); Needs to be completed - defaults to Do
				// Nothing
				autonPlan0_DoNothing();
				autonPlan = 6.0;

			} else if ((autonPlanOne == 0 && FieldConfig == 1) || (autonPlanTwo == 0 && FieldConfig == 2)
					|| (autonPlanThree == 0 && FieldConfig == 3) || (autonPlanFour == 0 && FieldConfig == 4))
			{
				autonPlan0_DoNothing();
				autonPlan = 0.0;

			} else if ((autonPlanOne == 7 && FieldConfig == 1) || (autonPlanTwo == 7 && FieldConfig == 2)
					|| (autonPlanThree == 7 && FieldConfig == 3) || (autonPlanFour == 7 && FieldConfig == 4))
			{
				autonPlan7_CrossTheLine();
				autonPlan = 7.0;

			} else
			{
				autonPlan0_DoNothing();
				autonPlan = 99.0;
			}
		} else
		{ // SWITCH for MIDDLE
			autonPlan67_MiddleSwitch();
			autonPlan = 67.0;
		}

		shiftedOld = shifted;
		AutonDashboardOut();
	} // end autonPeroidic

	boolean AllianceSwitchPosition()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0)
		{
			if (gameData.charAt(0) == 'L')
			{
				allianceSwitch = false;
			} else
			{
				allianceSwitch = true;
			}
		}

		return allianceSwitch;
	}

	int FieldConfiguration()
	{
		int output = 0;

		if (autonPosition == 0)
		{ // left
			if (AllianceSwitchPosition() == false && AllianceScalePosition() == false)
			{
				output = 1;
			} else if (AllianceSwitchPosition() == true && AllianceScalePosition() == true)
			{
				output = 2;
			} else if (AllianceSwitchPosition() == true && AllianceScalePosition() == false)
			{
				output = 3;
			} else if (AllianceSwitchPosition() == false && AllianceScalePosition() == true)
			{
				output = 4;
			}
		} else if (autonPosition == 1)
		{ // middle
			if (AllianceSwitchPosition() == false && AllianceScalePosition() == false)
			{ // left left
				output = 1;
			} else if (AllianceSwitchPosition() == true && AllianceScalePosition() == true)
			{ // right right
				output = 2;
			} else if (AllianceSwitchPosition() == true && AllianceScalePosition() == false)
			{ // right left
				output = 3;
			} else if (AllianceSwitchPosition() == false && AllianceScalePosition() == true)
			{ // left right
				output = 4;
			}
		} else
		{ // right
			if (AllianceSwitchPosition() == true && AllianceScalePosition() == true)
			{
				output = 1;
			} else if (AllianceSwitchPosition() == false && AllianceScalePosition() == false)
			{
				output = 2;
			} else if (AllianceSwitchPosition() == false && AllianceScalePosition() == true)
			{
				output = 3;
			} else if (AllianceSwitchPosition() == true && AllianceScalePosition() == false)
			{
				output = 4;
			}
		}

		return output;
	}

	boolean AllianceScalePosition()
	{
		String gameData;
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0)
		{
			if (gameData.charAt(1) == 'L')
			{
				allianceScale = false;
			} else
			{
				allianceScale = true;
			}
		}

		return allianceScale;
	}

	boolean autonDriveFinished(double distance, double angle)
	{
		m_drivetrain.SetPIDSetpoint(distance, angle);// m_drivetrain.getYaw());
		if (shifted == true)
		{
			m_drivetrain.UpdateDriveHIPID();
		} else
		{
			m_drivetrain.UpdateDrivePID();
		}
		if (Math.abs(m_drivetrain.GetDistancePIDError()) < 3.0)
		{
			m_drivetrain.ZeroDistance();
			m_drivetrain.DisablePID();
			m_drivetrain.ResetAnglePID();
			m_drivetrain.ResetDistancePID();
			return true;
		} else
		{
			m_drivetrain.EnablePID();
			return false;
		}
	}

	void autonPlan1_2xScale()
	{
		switch (m_autonCase)
		{
		case 0: // goForward
			if (m_drivetrain.GetAverageDistance() > 180.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_highScale);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else if (m_drivetrain.GetAverageDistance() > 50.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
			}

			if (Math.abs(m_drivetrain.GetAverageDistance()) < 40.0)
			{ // accel mopro
				dsf = Math.abs(m_drivetrain.GetAverageDistance() + 0.1);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (Math.abs(m_drivetrain.GetDistancePIDError()) < 40.0)
			{ // decel mopro
				dsf = Math.abs(m_drivetrain.GetDistancePIDError() + 0.25);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (autonDriveFinished(m_autonScaleForwardDistance, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 1: // turn
			m_elevator.magicalElevator(Elevator.button.auto_highScale);

			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_drivetrain.SetShift(false);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2: // intake down
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);

			m_drivetrain.UpdateDrivePIDOutput(0.4);

			if (autonDriveFinished(m_autonScaleDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}

			break;
		case 3: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if ((m_intake.GetWristAngle() > 15.0) && (Math.abs(m_elevator.getCurrentPosition()) > 32000.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 4: // spit
			if (m_autonSpitTimer.get() > 0.9)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
				m_autonCase++;
			} else
			{
				m_intake.FastShoot(false);
				m_intake.RunIntake(0.8);
			}
			break;
		case 5: // back up
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (autonDriveFinished(m_autonScaleBackUpDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 6: // turn
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_elevator.magicalElevator(Elevator.button.auto_package);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 7: // drive2bonusCube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_drivetrain.UpdateDrivePIDOutput(0.6);
			m_drivetrain.UpdateTurnPIDOutput(0.5);

			if (m_drivetrain.GetAverageDistance() > 20)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(false);
			} else
			{
				m_intake.OpenClaw(false);
				m_intake.NeutralClaw(true);
			}

			if (autonDriveFinished(m_autonScaleBonusCubeDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 8: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_autonCase++;
			break;
		case 9: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(true);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 10: // Sucking in bonus cube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_intake.RunIntake(-1.0);
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 11: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_intake.RunIntake(-1.0);
				m_autonCase++;
			}
			break;
		case 12:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 13:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_autonRollinTimer.get() > 0.4)
			{
				m_intake.NeutralClaw(false);
				m_intake.OpenClaw(false);
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.NeutralClaw(true);
				m_intake.OpenClaw(true);
				m_intake.RunIntake(-1.0);
			}
			break;
		case 14:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(0.0);
			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 15:
			if (m_autonRollinTimer.get() > 0.4)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			}
			break;
		case 16: // Clamp claw and finish backing up
			m_intake.RunIntake(0.0);
			m_intake.NeutralClaw(false);
			m_intake.OpenClaw(false);
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (autonDriveFinished(m_autonScaleBonusCubeBackupDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 17: // turn2scale
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_intake.RunIntake(0.0);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeBackupAngle, 0.8, 0.5) == true)
			{
				m_autonCase++;
			}
			break;
		case 18:
			m_intake.RunIntake(0.0);
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if ((m_intake.GetWristAngle() > 15.0) && (Math.abs(m_elevator.getCurrentPosition()) > 30000.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 19: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (autonDriveFinished(m_autonScaleBonusCubeForwardDistance, m_autonScaleBonusCubeBackupAngle) == true)
			{
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 20: // spit!
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_autonSpitTimer.get() > 1.0)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.4);
			}
			break;
		case 21: // prepare4zero
			m_intake.RunIntake(0.0);
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_autonCase++;
			break;
		case 22: // zeroTimerStart
			m_drivetrain.UpdateDrivePIDOutput(0.4);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeBackupAngle) == true)
			{
				m_autonCase++;
			}
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_autonZeroWristTimer.stop();
			m_autonZeroWristTimer.reset();
			m_autonZeroWristTimer.start();
			break;
		case 23: // reZero
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			if (m_autonZeroWristTimer.get() > 1.0)
			{
				m_intake.WristManual(0.0);
				m_autonCase++;
			} else
			{
				m_intake.WristManual(0.3);
			}
			break;
		case 24:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			break;

		} /* end scale same side auton */
	}

	void autonPlan2_SameSideSw()
	{
		switch (m_autonCase)
		{
		case 0: // Drive Forward
			m_drivetrain.SetShift(false);
			if (autonDriveFinished(m_autonSwitchForwardDistance, 0) == true)
			{
				m_autonCase++;
			} else
			{
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
				m_elevator.magicalElevator(Elevator.button.auto_package);
			}
			if (m_drivetrain.GetDistancePIDError() < 15.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.6);
			} else if (m_drivetrain.GetAverageDistance() < 20.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.4 + (m_drivetrain.GetAverageDistance() / 20.0) * 0.4);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}
			break;
		case 1: // Turn to switch
			m_elevator.magicalElevator(Elevator.button.auto_package);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);

			if (m_drivetrain.SweetTurnFinished(m_autonSwitchTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);

			m_drivetrain.UpdateDrivePIDOutput(0.6);

			if (autonDriveFinished(m_autonSwitchDistance, m_autonSwitchTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 3:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_intake.GetWristAngle() > 15.0)
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 4: // Spit
			if (m_autonSpitTimer.get() > 1.5)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
				// m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.6);
			}
			break;
		case 5: // do not execute following code until steal-the-cube is refined
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (autonDriveFinished(-m_autonSwitchDistance, m_autonSwitchTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 6:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (m_drivetrain.SweetTurnFinished(0, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 7:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(m_autonSwitchForwardDistance2, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 8:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (m_drivetrain.SweetTurnFinished(m_autonSwitchBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 9: // drive2bonusCube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_drivetrain.UpdateDrivePIDOutput(0.6);
			m_drivetrain.UpdateTurnPIDOutput(0.5);

			if (m_drivetrain.GetAverageDistance() > 20)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(false);
			} else
			{
				m_intake.OpenClaw(false);
				m_intake.NeutralClaw(true);
			}

			if (autonDriveFinished(m_autonSwitchBonusCubeDistance, m_autonSwitchBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 10: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_autonCase++;
			break;
		case 11: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(true);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 12:
			SmartDashboard.putNumber("Intake Slave Output", m_intake.GetIntakeSlavePercentOutput());
			SmartDashboard.putNumber("Intake Master Output", m_intake.GetIntakeMasterPercentOutput());

			m_intake.RunIntake(-1.0);
			if (m_autonRollinTimer.get() > 0.5)
			{
				// m_intake.RunIntake(0.0);
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 13:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(m_autonSwitchBonusCubeBackupDistance, m_autonSwitchBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 14:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (m_drivetrain.SweetTurnFinished(m_autonSwitchCTFAngle, 1, 1))
			{
				m_autonCase++;
			}
			break;
		case 15:
			if (autonDriveFinished(m_autonSwitchCTFDistance, 0) == true)
			{
				m_autonCase++;
			} else
			{
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
				m_elevator.magicalElevator(Elevator.button.auto_carry);
			}
			if (m_drivetrain.GetDistancePIDError() < 15.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.6);
			} else if (m_drivetrain.GetAverageDistance() < 20.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.4 + (m_drivetrain.GetAverageDistance() / 20.0) * 0.4);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			break;
		}
	}

	void autonPlan3_CrossTheFieldSc()
	{
		switch (m_autonCase)
		{
		case 0: /* Drive past switch */
			m_drivetrain.SetShift(false);
			if (autonDriveFinished(m_autonScaleForwardDistance, 0) == true)
			{
				m_autonCase++;
			} else
			{
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
				m_elevator.magicalElevator(Elevator.button.auto_package);
			}
			if (m_drivetrain.GetDistancePIDError() < 15.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.6);
			} else if (m_drivetrain.GetAverageDistance() < 20.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.4 + (m_drivetrain.GetAverageDistance() / 20.0) * 0.4);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			break;
		case 1: /* Turn to cross field */
			m_elevator.magicalElevator(Elevator.button.auto_package);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}

			break;
		case 2: /* Cross field */
			if (m_drivetrain.GetAverageDistance() > 225.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.4 + (m_drivetrain.GetAverageDistance() / 20.0) * 0.4);
				m_elevator.magicalElevator(Elevator.button.auto_highScale);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else if (m_drivetrain.GetAverageDistance() > 120.0)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
				m_drivetrain.UpdateTurnPIDOutput(0.8);
			}
			if (autonDriveFinished(m_autonScaleCrossFieldDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;

		case 3: // Turn to Switch
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle2, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 4: // Drive to Scale
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			m_drivetrain.UpdateDrivePIDOutput(0.5);

			if (autonDriveFinished(m_autonScaleDistance, m_autonScaleTurnAngle2) == true)
			{
				m_autonCase++;
			}
			break;
		case 5: // Shoot!
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (Math.abs(m_elevator.getCurrentPosition()) > 30000.0 && (m_intake.GetWristAngle() > 15.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 6:
			if (m_autonSpitTimer.get() > 1.5)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
				m_autonCase++;
			} else
			{
				m_intake.FastShoot(true);
				m_intake.RunIntake(0.67);
			}
			break;
		case 7:
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (autonDriveFinished(m_autonScaleBackUpDistance, m_autonScaleTurnAngle2) == true)
			{
				m_autonCase++;
			}
			break;
		case 8:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_elevator.magicalElevator(Elevator.button.auto_package);

			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 9:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_autonZeroWristTimer.stop();
			m_autonZeroWristTimer.reset();
			m_autonZeroWristTimer.start();
			m_autonCase++;
			break;
		case 10:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			if (m_autonZeroWristTimer.get() > 1.0)
			{
				m_intake.WristManual(0.0);
				m_autonCase++;
			} else
			{
				m_intake.WristManual(0.3);
			}
			break;
		}
	}

	void autonPlan4_SameSideScSw()
	{
		switch (m_autonCase)
		{
		case 0: // goForward
			if (m_drivetrain.GetAverageDistance() > 180.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_highScale);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else if (m_drivetrain.GetAverageDistance() > 50.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
			}

			if (Math.abs(m_drivetrain.GetAverageDistance()) < 40.0)
			{ // accel mopro
				dsf = Math.abs(m_drivetrain.GetAverageDistance() + 0.1);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (Math.abs(m_drivetrain.GetDistancePIDError()) < 40.0)
			{ // decel mopro
				dsf = Math.abs(m_drivetrain.GetDistancePIDError() + 0.25);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (autonDriveFinished(m_autonScaleForwardDistance, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 1: // turn
			m_elevator.magicalElevator(Elevator.button.auto_highScale);

			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_drivetrain.SetShift(false);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2: // intake down
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonBack);

			m_drivetrain.UpdateDrivePIDOutput(0.4);

			if (autonDriveFinished(m_autonScaleDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}

			break;
		case 3: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonBack);
			if (Math.abs(m_elevator.getCurrentPosition()) > 32000.0 && (m_intake.GetWristAngle() > 22.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 4: // spit
			if (m_autonSpitTimer.get() > 0.9)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonBack);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.6);
			}
			break;
		case 5: // back up
			// m_elevator.magicalElevator(Elevator.button.auto_highScale);
			// m_intake.magicalWrist(Intake.WristPosition.autonTen);
			//
			// if (autonDriveFinished(m_autonScaleBackUpDistance, m_autonScaleTurnAngle) ==
			// true){
			// m_autonCase++;
			// }
			break;
		case 6: // turn
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_elevator.magicalElevator(Elevator.button.auto_package);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 7: // drive2bonusCube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_drivetrain.UpdateDrivePIDOutput(0.6);
			m_drivetrain.UpdateTurnPIDOutput(0.5);

			if (m_drivetrain.GetAverageDistance() > 20)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(false);
			} else
			{
				m_intake.OpenClaw(false);
				m_intake.NeutralClaw(true);
			}

			if (autonDriveFinished(m_autonScaleBonusCubeDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 8: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_autonCase++;
			break;
		case 9: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(true);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 10:

			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_intake.RunIntake(-1.0);
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 11: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_intake.RunIntake(-1.0);
				m_autonCase++;
			}
			break;
		case 12:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 13:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.NeutralClaw(false);
				m_intake.OpenClaw(false);
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.NeutralClaw(true);
				m_intake.OpenClaw(true);
				m_intake.RunIntake(-1.0);
			}
			break;
		case 14:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(0.0);
			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 15:
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			}
			break;
		case 16:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (Math.abs(m_elevator.getCurrentPosition()) > 4500.0)
			{
				m_autonCase++;
			}
			break;
		case 17:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (autonDriveFinished(m_autonScaleSmallGoForwardDistance, m_autonScaleBonusCubeAngle))
			{
				m_autonCase++;
			}
			break;
		case 18:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_intake.GetWristAngle() > 22.0)
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 19: // spit!
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_autonSpitTimer.get() > 1.0)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.6);
			}
			break;

		} /* end scale same side auton */
	}

	void autonPlan5_2xScaleCompatible()
	{
		switch (m_autonCase)
		{
		case 0: // goForward
			if (m_drivetrain.GetAverageDistance() > 180.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_highScale);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else if (m_drivetrain.GetAverageDistance() > 50.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
			}

			if (Math.abs(m_drivetrain.GetAverageDistance()) < 40.0)
			{ // accel mopro
				dsf = Math.abs(m_drivetrain.GetAverageDistance() + 0.1);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (Math.abs(m_drivetrain.GetDistancePIDError()) < 40.0)
			{ // decel mopro
				dsf = Math.abs(m_drivetrain.GetDistancePIDError() + 0.25);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (autonDriveFinished(m_autonScaleForwardDistance, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 1: // turn
			m_elevator.magicalElevator(Elevator.button.auto_highScale);

			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_drivetrain.SetShift(false);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2: // intake down
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);

			m_drivetrain.UpdateDrivePIDOutput(0.4);

			if (autonDriveFinished(m_autonScaleDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}

			break;
		case 3: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if ((m_intake.GetWristAngle() > 15.0) && (Math.abs(m_elevator.getCurrentPosition()) > 32000.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 4: // spit
			if (m_autonSpitTimer.get() > 0.9)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
				m_autonCase++;
			} else
			{
				m_intake.FastShoot(false);
				m_intake.RunIntake(0.8);
			}
			break;
		case 5: // back up
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (autonDriveFinished(m_autonScaleBackUpDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 6: // turn
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_elevator.magicalElevator(Elevator.button.auto_package);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 7: // drive2bonusCube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_drivetrain.UpdateDrivePIDOutput(0.6);
			m_drivetrain.UpdateTurnPIDOutput(0.5);

			if (m_drivetrain.GetAverageDistance() > 20)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(false);
			} else
			{
				m_intake.OpenClaw(false);
				m_intake.NeutralClaw(true);
			}

			if (autonDriveFinished(m_autonScaleBonusCubeDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 8: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_autonCase++;
			break;
		case 9: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(true);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 10: // Sucking in bonus cube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_intake.RunIntake(-1.0);
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 11: // slow backup
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_intake.RunIntake(-1.0);
				m_autonCase++;
			}
			break;
		case 12:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleCompatibleBonusAngle, 1, 0.4) == true)
			{
				m_autonCase++;
			}
			break;
		case 13:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 14:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_autonRollinTimer.get() > 0.4)
			{
				m_intake.NeutralClaw(false);
				m_intake.OpenClaw(false);
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.NeutralClaw(true);
				m_intake.OpenClaw(true);
				m_intake.RunIntake(-1.0);
			}
			break;
		case 15:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(0.0);
			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 16:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_intake.RunIntake(0.0);
			m_autonCase++;
			break;
		case 17: // Clamp claw and backup
			m_intake.RunIntake(0.0);
			m_intake.NeutralClaw(false);
			m_intake.OpenClaw(false);
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (autonDriveFinished(m_autonScaleCompatibleDistance1, m_autonScaleCompatibleBonusAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 18: // turn to spit
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_intake.RunIntake(0.0);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeBackupAngle, 0.8, 0.4) == true)
			{
				m_autonCase++;
			}
			break;
		case 19:
			m_intake.RunIntake(0.0);
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if ((m_intake.GetWristAngle() > 15.0) && (Math.abs(m_elevator.getCurrentPosition()) > 30000.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 20: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			// if(autonDriveFinished(m_autonScaleBonusCubeForwardDistance,
			// m_autonScaleBonusCubeBackupAngle) == true){
			m_autonSpitTimer.start();
			m_autonCase++;
			// }
			break;
		case 21: // spit!
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_autonSpitTimer.get() > 1.0)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.4);
			}
			break;
		case 22: // prepare4zero
			m_intake.RunIntake(0.0);
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_autonCase++;
			break;
		case 23: // zeroTimerStart
			m_drivetrain.UpdateDrivePIDOutput(0.4);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeBackupAngle) == true)
			{
				m_autonCase++;
			}
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			m_autonZeroWristTimer.stop();
			m_autonZeroWristTimer.reset();
			m_autonZeroWristTimer.start();
			break;
		case 24: // reZero
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			if (m_autonZeroWristTimer.get() > 1.0)
			{
				m_intake.WristManual(0.0);
				m_autonCase++;
			} else
			{
				m_intake.WristManual(0.3);
			}
			break;
		case 25:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			break;

		} /* end scale same side auton */
	}

	void autonPlan6_SameSideScOppSideSw()
	{
		switch (m_autonCase)
		{
		case 0: // goForward
			if (m_drivetrain.GetAverageDistance() > 180.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_highScale);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else if (m_drivetrain.GetAverageDistance() > 50.0)
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
			} else
			{
				m_elevator.magicalElevator(Elevator.button.auto_package);
				m_intake.magicalWrist(Intake.WristPosition.autonUp);
			}

			if (Math.abs(m_drivetrain.GetAverageDistance()) < 40.0)
			{ // accel mopro
				dsf = Math.abs(m_drivetrain.GetAverageDistance() + 0.1);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (Math.abs(m_drivetrain.GetDistancePIDError()) < 40.0)
			{ // decel mopro
				dsf = Math.abs(m_drivetrain.GetDistancePIDError() + 0.25);
				m_drivetrain.UpdateDrivePIDOutput((0.6 / 40.0) * dsf);
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
			}

			if (autonDriveFinished(m_autonScaleForwardDistance, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 1: // turn
			m_elevator.magicalElevator(Elevator.button.auto_highScale);

			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			m_drivetrain.SetShift(false);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2: // intake down
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);

			m_drivetrain.UpdateDrivePIDOutput(0.4);

			if (autonDriveFinished(m_autonScaleDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}

			break;
		case 3: // startSpitTimer
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if ((m_intake.GetWristAngle() > 15.0) && (Math.abs(m_elevator.getCurrentPosition()) > 32000.0))
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 4: // spit
			if (m_autonSpitTimer.get() > 0.9)
			{
				m_intake.RunIntake(0.0);
				m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
				m_autonCase++;
			} else
			{
				m_intake.FastShoot(false);
				m_intake.RunIntake(0.8);
			}
			break;
		case 5: // back up
			m_elevator.magicalElevator(Elevator.button.auto_highScale);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (autonDriveFinished(m_autonScaleBackUpDistance, m_autonScaleTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 6: // turn
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_elevator.magicalElevator(Elevator.button.auto_package);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 7: // drive2bonusCube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.RunIntake(-1.0);
			m_drivetrain.UpdateDrivePIDOutput(0.6);
			m_drivetrain.UpdateTurnPIDOutput(0.5);

			if (m_drivetrain.GetAverageDistance() > 20)
			{ // this distance will be much larger
				m_intake.magicalWrist(Intake.WristPosition.autonDown);
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(false);
			} else
			{
				m_intake.magicalWrist(Intake.WristPosition.autonTen);
				m_intake.OpenClaw(false);
				m_intake.NeutralClaw(true);
			}

			if (autonDriveFinished(m_autonScaleBonusCubeDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 8: // turn to bonus cube, cube 2 or 3 on sc side
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubePickupAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 9: // drive forward slightly to pick up cube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonScaleBonusCubePickupDistance, m_autonScaleBonusCubePickupAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 10: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			m_autonCase++;
			break;
		case 11: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(true);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 12: // Sucking in bonus cube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_intake.RunIntake(-1.0);
			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.OpenClaw(true);
				m_intake.NeutralClaw(true);
				m_autonCase++;
			}
			break;
		case 13: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonScaleSmallBackUpDistance, m_autonScaleBonusCubeAngle) == true)
			{
				m_intake.RunIntake(-1.0);
				m_autonCase++;
			}
			break;
		case 14:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 15:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_autonRollinTimer.get() > 0.4)
			{
				m_intake.NeutralClaw(false);
				m_intake.OpenClaw(false);
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.NeutralClaw(true);
				m_intake.OpenClaw(true);
				m_intake.RunIntake(-1.0);
			}
			break;
		case 16:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(0.0);
			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();
			m_autonCase++;
			break;
		case 17:
			if (m_autonRollinTimer.get() > 0.4)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			}
			break;
		case 18:
			m_intake.RunIntake(0.0);
			m_intake.NeutralClaw(false);
			m_intake.OpenClaw(false);
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);

			if (m_drivetrain.SweetTurnFinished(m_autonScaleBonusCubeAngle, 1, 0.8) == true)
			{
				m_autonCase++;
			}
			break;
		case 19:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (Math.abs(m_elevator.getCurrentPosition()) > 4500.0)
			{
				m_autonCase++;
			}
			break;
		case 20:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (autonDriveFinished(m_autonScaleSmallGoForwardDistance, m_autonScaleBonusCubeAngle))
			{
				m_autonCase++;
			}
			break;
		case 21:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_intake.GetWristAngle() > 22.0)
			{
				m_autonSpitTimer.stop();
				m_autonSpitTimer.reset();
				m_autonSpitTimer.start();
				m_autonCase++;
			}
			break;
		case 22: // spit!
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonMiddle);
			if (m_autonSpitTimer.get() > 1.0)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.6);
			}
			break;
		}
	} /* end scale same side opposite side switch auton */

	void autonPlan7_CrossTheLine()
	{
		switch (m_autonCase)
		{
		case 0:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(160, 0) == true)
			{
				m_autonCase++;
			}
			break;
		}
	}

	void autonPlan67_MiddleSwitch()
	{
		switch (m_autonCase)
		{
		case 0:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(15.0, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 1:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (m_drivetrain.SweetTurnFinished(m_autonSwitchTurnAngle, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 2:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(m_autonSwitchDistance, m_autonSwitchTurnAngle) == true)
			{
				m_autonCase++;
			}
			break;
		case 3:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (m_drivetrain.SweetTurnFinished(0, 1, 1) == true)
			{
				m_autonCase++;
			}
			break;
		case 4:
			m_intake.magicalWrist(Intake.WristPosition.autonBack);
			m_elevator.magicalElevator(Elevator.button.auto_carry);

			if (autonDriveFinished(m_autonSwitchForwardDistance, 0) == true)
			{
				m_autonCase++;

				m_autonTimer.stop();
				m_autonTimer.reset();
				m_autonTimer.start();
			}
			break;
		case 5:
			m_intake.magicalWrist(Intake.WristPosition.autonBack);
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			if (m_autonTimer.get() > 0.2)
			{
				m_autonTimer.stop();
				m_autonTimer.reset();
				m_autonTimer.start();
				m_autonCase++;
			}
			break;
		case 6:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonBack);

			if (m_autonTimer.get() > 0.5)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.7);
			}
			break;
		case 7:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (autonDriveFinished(m_autonSwitchBackUpDistance, 0) == true)
			{
				m_autonCase++;
			}
			break;
		case 8:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_drivetrain.SweetTurnFinished(m_autonSwitchBonusCubeAngle, 1, 0.8) == true)
			{
				m_autonCase++;
			}
			break;
		case 9:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);
			if ((autonDriveFinished(m_autonSwitchBonusCubeDistance, 0) == true) && (m_intake.GetWristAngle() > 85.0))
			{
				m_autonCase++;
			}
			break;
		case 10: // rollin'
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			m_intake.RunIntake(-1.0);

			m_intake.NeutralClaw(true);
			m_intake.OpenClaw(false); // true

			m_autonRollinTimer.stop();
			m_autonRollinTimer.reset();
			m_autonRollinTimer.start();

			m_autonCase++;
			break;
		case 11: // Sucking in bonus cube
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (m_autonRollinTimer.get() > 1.0)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(-1.0);
			}

			if (m_autonRollinTimer.get() > 0.5)
			{
				m_intake.NeutralClaw(false);
				m_intake.OpenClaw(false);
			}
			break;
		case 12: // startRollin'Timer
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonDown);
			if (autonDriveFinished(m_autonSwitchSmallBackUpDistance, m_autonSwitchBonusCubeAngle) == true)
			{
				m_drivetrain.UpdateDrivePIDOutput(0.8);
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_drivetrain.UpdateDrivePIDOutput(0.4);
				m_intake.RunIntake(0.0);
			}
			break;
		case 13:
			m_elevator.magicalElevator(Elevator.button.auto_pickup);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (m_drivetrain.SweetTurnFinished(m_autonSwitchTurnToScale1, 1, 0.4) == true)
			{ // third value is 0.67 for sans-2-cube
				m_autonCase++;
			}
			break;
		case 14: // --------------------------------------------------------------------------------------
					// if you don't want to 2-cube switch, COMMENT THIS CASE OUT
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonTen);
			if (autonDriveFinished(m_autonSwitchDriveToScale1, m_autonSwitchTurnToScale1) == true)
			{
				m_autonCase++;
			}
			break;
		case 15:
			m_intake.magicalWrist(Intake.WristPosition.autonBack);
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			if (m_autonTimer.get() > 0.2)
			{
				m_autonTimer.stop();
				m_autonTimer.reset();
				m_autonTimer.start();
				m_autonCase++;
			}
			break;
		case 16:
			m_elevator.magicalElevator(Elevator.button.auto_carry);
			m_intake.magicalWrist(Intake.WristPosition.autonBack);

			if (m_autonTimer.get() > 0.4)
			{
				m_intake.RunIntake(0.0);
				m_autonCase++;
			} else
			{
				m_intake.RunIntake(0.35);
			}
			break;
		}
	}

	void autonPlan0_DoNothing()
	{
		switch (m_autonCase)
		{
		case 0:
			m_intake.magicalWrist(Intake.WristPosition.autonUp);
			if (autonDriveFinished(0, 0) == true)
			{
				m_autonCase++;
			}
			break;
		}
	}

	void TeleopInit()
	{
		m_drivetrain.DisablePID();

		// ------------------------------------------------------------------------//
		// Reset After Autonomous
		// ------------------------------------------------------------------------//
		m_drivetrain.ArcadeDrive(0, 0);
		m_intake.WristManual(0);
		m_intake.RunIntake(0);
		m_climber.manualClimber(0);
		m_drivetrain.SetRamp(0.0);
		m_elevator.manualElevator(0.0);

		servoRun = false;

	}

	void TeleopPeriodic()
	{
		m_intake.ReadSensors();
		// timeVar = std::chrono::high_resolution_clock::now();
		// timeNow = 0 + timeVar;
		// SmartDashboard.putNumber("time", timeNow);

		m_colorSensor.read(I2C_READ, 1, buffer);
		SmartDashboard.putNumber("Color Number:", new BigInteger(buffer).intValue());

		SmartDashboard.putNumber("Intake Speed", m_intake.GetIntakeMasterPercentOutput());

		// intake
		// driver controls are opposite operator. So if driver.LT and operator.RT are
		// pressed, they are commanding the same function.

		if (m_driver.ButtonRT())
		{
			m_intake.FastShoot(true);
			m_intake.GetIntakeDesiredSpeed();
			m_intake.RunIntake(0.65);
		} else if (m_operator.ButtonLT())
		{
			m_intake.FastShoot(false);
			m_intake.GetIntakeDesiredSpeed();
			m_intake.RunIntake(0.65);
		} else if (m_operator.ButtonRT())
		{
			m_intake.GetIntakeDesiredSpeed();
			m_intake.RunIntake(-0.8);
		} else
		{
			m_intake.RunIntake(0.0);
		}

		// wrist and ramp

		// drivetrain

		/* Schang mods for anti-tip start */
		m_drivetrain.ArcadeDrive(l_mapJoyStick.GetMappedValue(-m_driver.AxisLY()),
				r_mapJoyStick.GetMappedValue(m_driver.AxisRX()));

		if (m_elevator.getCurrentPosition() > antiTipHigh)
		{
			driverLY_Final = (antiTipSclFx * l_mapJoyStick.GetMappedValue(-m_driver.AxisLY()))
					+ ((1.0 - antiTipSclFx) * driverLY_Final);
		} else
		{
			driverLY_Final = l_mapJoyStick.GetMappedValue(-m_driver.AxisLY());
		}

		m_drivetrain.ArcadeDrive(driverLY_Final, r_mapJoyStick.GetMappedValue(m_driver.AxisRX()));

		/* Schang mods for anti-tip end */

		if (m_driver.ButtonLB())
		{
			m_drivetrain.SetShift(true);
		} else
		{
			m_drivetrain.SetShift(false);
		}

		// claw ||| wolverine
		// >=(o_o)=<``````````````````````````````````````````````````````````````````
		if (m_operator.ButtonRB() || m_driver.ButtonRB())
		{
			m_intake.NeutralClaw(false);
			m_intake.OpenClaw(true);
		} else if (m_operator.ButtonBack())
		{
			m_intake.OpenClaw(true);
			m_intake.NeutralClaw(true);
		} else
		{
			m_intake.NeutralClaw(false);
			m_intake.OpenClaw(false);
		}

		// elevator and climber
		if (m_operator.ButtonLB())
		{
			if (m_operator.ButtonY())
			{
				m_elevator.magicalElevator(Elevator.button.highScale);
				m_intake.magicalWrist(Intake.WristPosition.Back);
			} else if (m_operator.ButtonB())
			{
				m_elevator.magicalElevator(Elevator.button.midScale);
				m_intake.magicalWrist(Intake.WristPosition.Back);
			} else if (m_operator.ButtonX())
			{
				m_intake.magicalWrist(Intake.WristPosition.Up);
				if (m_intake.wristAtPosition() == true)
				{
					m_elevator.magicalElevator(Elevator.button.pack);
				}
			} else if (m_operator.ButtonA())
			{
				m_elevator.magicalElevator(Elevator.button.lowScale);
				m_intake.magicalWrist(Intake.WristPosition.Back);
			} else
			{
				m_elevator.manualElevator(0.0);
				m_intake.WristManual(0.0);
			}

			if (Math.abs(m_operator.AxisRY()) > 0.2)
			{
				m_climber.manualClimber((float) -m_operator.AxisRY());

				climberCase = 0;
				climberSafe = false;
			} else if (m_driver.getPOV() == 225 || m_driver.getPOV() == 180 || m_driver.getPOV() == 135)
			{

				if (m_climber.getCurrentPosition() < 13443.2820513)
				{
					m_climber.manualClimber(-0.5f);
				} else
				{
					m_climber.manualClimber(-1.0f);
				}

			} else if (m_driver.AxisLT() > 0.5)
			{
				m_climber.manualClimber(1.0f);
				climberCase = 0;
				climberSafe = false;
			} else
			{
				m_climber.manualClimber(0);
			}
		} else
		{
			if (m_operator.ButtonA())
			{
				m_elevator.magicalElevator(Elevator.button.pickup);
				m_intake.magicalWrist(Intake.WristPosition.Down);
			} else if (m_operator.ButtonB())
			{
				m_elevator.magicalElevator(Elevator.button.carry);
				m_intake.magicalWrist(Intake.WristPosition.Middle);
			} else if (m_operator.ButtonX())
			{
				m_intake.magicalWrist(Intake.WristPosition.Up);
				if (m_intake.wristAtPosition() == true)
				{
					m_elevator.magicalElevator(Elevator.button.pack);
				}
			} else if (m_operator.ButtonY())
			{
				m_elevator.magicalElevator(Elevator.button.slamDunk);
				m_intake.magicalWrist(Intake.WristPosition.HighHigh);
			} else if (m_operator.getPOV() == 315 || m_operator.getPOV() == 0 || m_operator.getPOV() == 45)
			{
				m_drivetrain.SetRatchet(0.5);
				m_drivetrain.SetRamp(1.0);
			} else if (m_operator.getPOV() == 225 || m_operator.getPOV() == 180 || m_operator.getPOV() == 135)
			{
				/*
				 * down, you need to put the servo to... turn the servo wait 100 ms pulse ramp
				 * 100 ms run ramp down
				 */
				switch (servoCase)
				{
				case 0:
					m_servoTimer.stop();
					m_servoTimer.reset();
					m_servoTimer.start();
					servoCase++;
					break;
				case 1:
					m_drivetrain.SetRatchet(0.0);
					if (m_servoTimer.get() > 0.1)
					{
						m_servoTimer.stop();
						m_servoTimer.reset();
						m_servoTimer.start();
						servoCase++;
					}
					break;
				case 2:
					m_drivetrain.SetRamp(1.0);
					if (m_servoTimer.get() > 0.1)
					{
						m_servoTimer.stop();
						m_servoTimer.reset();
						m_servoTimer.start();
						m_drivetrain.SetRamp(-1.0);
						servoCase++;
					}
					break;
				case 3:
					m_drivetrain.SetRamp(-1.0);
					servoCase++;
					break;
				}
			} else
			{
				m_elevator.manualElevator(m_operator.AxisRY() * 0.5);

				if ((Math.abs(m_operator.AxisLY()) > 0.2))
				{
					m_intake.WristManual(-m_operator.AxisLY());
				} else
				{
					m_intake.WristManual(0.0);
				}

				m_drivetrain.SetRatchet(0.5);
				m_drivetrain.SetRamp(0.0);
				servoCase = 0;
			}

			if (m_driver.getPOV() == 225 || m_driver.getPOV() == 180 || m_driver.getPOV() == 135)
			{
				if (m_climber.getCurrentPosition() < 13443.2820513)
				{
					m_climber.manualClimber(-0.5f);
				} else
				{
					m_climber.manualClimber(-1.0f);
				}
			} else if (m_driver.AxisLT() > 0.5)
			{
				m_climber.manualClimber(1.0f);
				climberCase = 0;
				climberSafe = false;
			} else
			{
				m_climber.manualClimber(0);
				climberCase = 0;
				climberSafe = false;
			}

		} // end elevator and climber

		SmartDashboard.putNumber("AxisLT", m_driver.AxisLT());

		if (m_operator.ButtonStart())
		{
			m_intake.ZeroWrist();
		}

		// --------------------------------------------------------------------------------------------//
		// Auton/Match Recording
		// --------------------------------------------------------------------------------------------//
		double matchTime;
		matchTime = DriverStation.getInstance().getMatchTime();
		SmartDashboard.putNumber("time", matchTime);
		SmartDashboard.putNumber("AutonCase", 99.0);

		// --------------------------------------------------------------------------------------------//
		// Output Recording
		// --------------------------------------------------------------------------------------------//

		SmartDashboard.putNumber("Output Drivetrain Left", m_drivetrain.GetLeftTalonPercentOutput());
		SmartDashboard.putNumber("Output Drivetrain Right", m_drivetrain.GetRightTalonPercentOutput());

		SmartDashboard.putNumber("Output Ramp", m_drivetrain.GetRampPercentOutput());

		SmartDashboard.putNumber("Output Intake", m_intake.GetIntakeDesiredSpeed());

		SmartDashboard.putNumber("Output Wrist", m_intake.GetWristPercentOutput());

		SmartDashboard.putNumber("Output Elevator", m_elevator.elevatorCommanded());
		SmartDashboard.putNumber("Output Climber", m_climber.climberCommanded());

		SmartDashboard.putNumber("Current - master", m_climber.getClimberMasterCurrent());
		SmartDashboard.putNumber("Current - slave", m_climber.getClimberSlaveCurrent());

		SmartDashboard.putNumber("WristAngle", m_intake.GetWristAngle());

		// --------------------------------------------------------------------------------------------//
		// Encoder Recording
		// --------------------------------------------------------------------------------------------//

		SmartDashboard.putNumber("Angle", m_drivetrain.GetYaw());
		SmartDashboard.putNumber("Elevator: currentPosition", m_elevator.getCurrentPosition());
		SmartDashboard.putNumber("Climber : currentPosition", m_climber.getCurrentPosition());
		SmartDashboard.putNumber("wristError", m_intake.GetWristError());
		SmartDashboard.putNumber("wristAtPosition", (m_intake.wristAtPosition()) ? 1.0 : 0.0);
		SmartDashboard.putNumber("Wrist: currentPosition", m_intake.GetWristActualPosition());

		// --------------------------------------------------------------------------------------------//
		// Joystick Recording
		// --------------------------------------------------------------------------------------------//
		/*
		 * SmartDashboard.putNumber("Driver LT", m_driver.AxisLT());
		 * SmartDashboard.putNumber("Driver RT", m_driver.AxisRT());
		 * 
		 * SmartDashboard.putNumber("Operator LT", m_operator.AxisLT());
		 * SmartDashboard.putNumber("Operator RT", m_operator.AxisRT());
		 * 
		 * SmartDashboard.putNumber("Driver LY", m_driver.AxisLY());
		 * SmartDashboard.putNumber("Driver RY", m_driver.AxisRY());
		 * 
		 * SmartDashboard.putNumber("Operator LY", m_operator.AxisLY());
		 * SmartDashboard.putNumber("Operator RY", m_operator.AxisRY());
		 * 
		 * SmartDashboard.putNumber("Driver LX", m_driver.AxisLX());
		 * SmartDashboard.putNumber("Driver RX", m_driver.AxisRX());
		 * 
		 * SmartDashboard.putNumber("Operator LX", m_operator.AxisLX());
		 * SmartDashboard.putNumber("Operator RX", m_operator.AxisRX());
		 * 
		 * SmartDashboard.putNumber("Driver A", m_driver.ButtonA());
		 * SmartDashboard.putNumber("Driver B", m_driver.ButtonB());
		 * 
		 * SmartDashboard.putNumber("Operator A", m_operator.ButtonA());
		 * SmartDashboard.putNumber("Operator B", m_operator.ButtonB());
		 * 
		 * SmartDashboard.putNumber("Driver X", m_driver.ButtonX());
		 * SmartDashboard.putNumber("Driver Y", m_driver.ButtonY());
		 * 
		 * SmartDashboard.putNumber("Operator X", m_operator.ButtonX());
		 * SmartDashboard.putNumber("Operator Y", m_operator.ButtonY());
		 * 
		 * SmartDashboard.putNumber("Driver Back", m_driver.ButtonBack());
		 * SmartDashboard.putNumber("Driver Start", m_driver.ButtonStart());
		 * 
		 * SmartDashboard.putNumber("Operator Back", m_operator.ButtonBack());
		 * SmartDashboard.putNumber("Operator Start", m_operator.ButtonStart());
		 * 
		 * SmartDashboard.putNumber("Driver POV", m_driver.GetPOV());
		 * SmartDashboard.putNumber("Operator POV", m_operator.GetPOV());
		 */

	} // end TeleopPeriodic

	void AutonDashboardOut()
	{

		SmartDashboard.putNumber("WristAngle", m_intake.WristAngle);

		SmartDashboard.putNumber("time", matchTime);
		SmartDashboard.putNumber("Auton Position", autonPosition);
		SmartDashboard.putNumber("AutonCase", m_autonCase);
		SmartDashboard.putNumber("AutonPlan", autonPlan);

		// --------------------------------------------------------------------------------------------//
		// Output Recording
		// --------------------------------------------------------------------------------------------//

		SmartDashboard.putNumber("Output Drivetrain Left", m_drivetrain.GetLeftTalonPercentOutput());
		SmartDashboard.putNumber("Output Drivetrain Right", m_drivetrain.GetRightTalonPercentOutput());

		SmartDashboard.putNumber("Output Ramp", m_drivetrain.GetRampPercentOutput());

		SmartDashboard.putNumber("Output Intake", m_intake.GetIntakeDesiredSpeed());

		SmartDashboard.putNumber("Output Wrist", m_intake.GetWristPercentOutput());

		SmartDashboard.putNumber("Output Elevator", m_elevator.elevatorCommanded());
		SmartDashboard.putNumber("Output Climber", m_climber.climberCommanded());
		// --------------------------------------------------------------------------------------------//
		// Encoder Recording
		// --------------------------------------------------------------------------------------------//

		SmartDashboard.putNumber("Angle", m_drivetrain.GetYaw());
		SmartDashboard.putNumber("Elevator: currentPosition", m_elevator.getCurrentPosition());
		SmartDashboard.putNumber("Climber : currentPosition", m_climber.getCurrentPosition());
		SmartDashboard.putNumber("Wrist : currentPosition", m_intake.GetWristActualPosition());

		// SmartDashboard.putNumber("Drivetrain: Left Distance",
		// m_drivetrain.GetLeftDistance());
		// SmartDashboard.putNumber("Drivetrain: Right Distance",
		// m_drivetrain.GetRightDistance());
		SmartDashboard.putNumber("Drivetrain: Average Distance", m_drivetrain.GetAverageDistance());
		SmartDashboard.putNumber("DrivetrainPID: DistanceSetpoint", m_drivetrain.GetDistanceSetpoint());
		SmartDashboard.putNumber("DrivetrainPID: DistanceError", m_drivetrain.GetDistancePIDError());
		SmartDashboard.putNumber("DrivetrainPID: DistancePIDSpeed", m_drivetrain.GetDistancePIDSpeed());
		SmartDashboard.putNumber("DistanceScaleFactor", dsf);

		SmartDashboard.putNumber("PigeonState", m_drivetrain.PigeonState());
		SmartDashboard.putNumber("HasRampReset", (m_drivetrain.HasRampReset()) ? 1 : 0);
		SmartDashboard.putNumber("HasGyroReset", (m_drivetrain.HasGyroReset()) ? 1 : 0);

		SmartDashboard.putNumber("Gyro Update", m_drivetrain.badPigeonCounter);
		SmartDashboard.putNumber("ElevatorError", m_elevator.GetElevatorError());
		SmartDashboard.putNumber("WristError", m_intake.GetWristError());
		SmartDashboard.putNumber("CubicShuffleActive", (m_intake.GetCubicShuffleActive()) ? 1 : 0);

		SmartDashboard.putNumber("Auton Type", autonPlanOne);
		SmartDashboard.putNumber("Auton Type 2", autonPlanTwo);
		SmartDashboard.putNumber("Auton Type 3", autonPlanThree);
		SmartDashboard.putNumber("Auton Type 4", autonPlanFour);

		// for normal use / calibration
		m_colorSensor.read(I2C_READ, 1, buffer);
		SmartDashboard.putNumber("Color Number:", new BigInteger(buffer).intValue());

		// UltraSonicRecording
		SmartDashboard.putNumber("UltraSonic Left", m_drivetrain.GetUltrasonicLeft());
		SmartDashboard.putNumber("UltraSonic Right", m_drivetrain.GetUltrasonicRight());
		SmartDashboard.putNumber("UltraSonic Rear", m_drivetrain.GetUltrasonicRear());

		// Spit Motor Output
		SmartDashboard.putNumber("Intake Slave Output", m_intake.GetIntakeSlavePercentOutput());
		SmartDashboard.putNumber("Intake Master Output", m_intake.GetIntakeMasterPercentOutput());

		SmartDashboard.putNumber("FieldConfig", FieldConfig);
	}

	void TeleopDrive()
	{

	}

	void TeleopData()
	{
		// SmartDashboard.putNumber("Yaw", m_drivetrain.getYaw());
	}

	void TestPeriodic()
	{
	}

}
