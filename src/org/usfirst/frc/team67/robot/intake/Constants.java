package org.usfirst.frc.team67.robot.intake;

public class Constants
{

	public static final int CLAW_SOLENOID = 2;
	public static final int CLAW_NEUTRAL_SOLENOID = 3;

	public static final int VICTOR_INTAKE_MASTER = 12;
	public static final int VICTOR_INTAKE_SLAVE = 6;
	public static final int TALON_WRIST = 13;

	public static final float SPEED_COEFF = .01f;

	public static final int PERCENT_SPEED = 1;
	public static final int PERCENT_SPEED_NEG = -1;
	public static final int DISABLED = 0;

	public static final float ANGLE_INTAKE_DOWN = 105.0f;
	public static final float ANGLE_INTAKE_MID = 35.0f;
	public static final float ANGLE_INTAKE_UP = 20.0f;
	public static final float ANGLE_INTAKE_BACK = 75.0f;
	public static final float ANGLE_INTAKE_TEN = 20.0f;
	public static final float ANGLE_INTAKE_SPIT = 35.0f;
	public static final float ANGLE_HIGH_HIGH = 32.9f;
	public static final float ANGLE_HIGH = 45.0f;

	public static final float ANGLE_INTAKE_AUTONMID = 35.0f;
	public static final float ANGLE_INTAKE_AUTONBACK = 42.5f;
	public static final float ANGLE_INTAKE_AUTONDOWN = 105.0f;
	public static final float ANGLE_INTAKE_AUTONUP = 20.0f;
	public static final float ANGLE_INTAKE_AUTONTEN = 20.0f;

	public static final float ANGLE_INTAKE_UP_P = 0.73f;
	public static final float ANGLE_INTAKE_UP_I = 0.001f;
	public static final float ANGLE_INTAKE_UP_D = 0.0f;
	public static final float ANGLE_INTAKE_UP_F = 0.88f;

	public static final float ANGLE_INTAKE_MID_P = 0.8f;
	public static final float ANGLE_INTAKE_MID_I = 0.0f;
	public static final float ANGLE_INTAKE_MID_D = 0.0f;
	public static final float ANGLE_INTAKE_MID_F = 0.88f;

	public static final float ANGLE_INTAKE_DOWN_P = 0.75f;
	public static final float ANGLE_INTAKE_DOWN_I = 0.0f;
	public static final float ANGLE_INTAKE_DOWN_D = 0.0f;
	public static final float ANGLE_INTAKE_DOWN_F = 0.85f;

	public static final double INTAKE_UNITS = 0.0617283950617284; // 90/1458

}
