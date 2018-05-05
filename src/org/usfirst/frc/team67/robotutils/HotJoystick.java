package org.usfirst.frc.team67.robotutils;

import edu.wpi.first.wpilibj.Joystick;

public class HotJoystick extends Joystick
{
	private boolean f_A, f_B, f_X, f_Y, f_LB, f_RB, f_Back, f_Start, f_L3, f_R3, f_LT, f_RT;
	private float db_LX, db_LY, db_LT, db_RT, db_RX, db_RY;

	public enum Button
	{
		A(1 << 0), B(1 << 1), X(1 << 2), Y(1 << 3), LB(1 << 4), RB(1 << 5), Back(1 << 6), Start(1 << 7), L3(1 << 8), R3(
				1 << 9), LT(1 << 10), RT(1 << 11);
		public final Integer Value;

		Button(int v)
		{
			Value = v;
		}
	}

	public enum Axis
	{
		LX(1 << 0), LY(1 << 1), LT(1 << 2), RT(1 << 3), RX(1 << 4), RY(1 << 5);

		public final Integer Value;

		Axis(int v)
		{
			Value = v;
		}
	}

	/******************************
	 * Constructor
	 ******************************/
	public HotJoystick(int port)
	{
		super(port);
		f_A = f_B = f_X = f_Y = f_LB = f_RB = f_Back = f_Start = f_L3 = f_R3 = f_LT = f_RT = false;
		db_LX = db_LY = db_LT = db_RT = db_RX = db_RY = (float) 0.0f;
	}

	/******************************
	 * HotJoystick
	 ******************************/
	/**
	 * Set Deadband
	 */
	public void SetDeadband(int channels, float value)
	{
		if ((channels & Axis.LX.Value) > 0)
		{
			db_LX = value;
		}
		if ((channels & Axis.LY.Value) > 0)
		{
			db_LY = value;
		}
		if ((channels & Axis.LT.Value) > 0)
		{
			db_LT = value;
		}
		if ((channels & Axis.RT.Value) > 0)
		{
			db_LX = value;
		}
		if ((channels & Axis.RX.Value) > 0)
		{
			db_RX = value;
		}
		if ((channels & Axis.RY.Value) > 0)
		{
			db_RY = value;
		}

	}

	/**
	 * Get Deadband
	 */
	public float GetDeadband(Axis axis)
	{
		if (axis == Axis.LX)
		{
			return db_LX;
		} else if (axis == Axis.LY)
		{
			return db_LY;
		} else if (axis == Axis.LT)
		{
			return db_LT;
		} else if (axis == Axis.RT)
		{
			return db_RT;
		} else if (axis == Axis.RX)
		{
			return db_RX;
		} else if (axis == Axis.RY)
		{
			return db_RY;
		} else
		{
			return 0.0f;
		}

	}

	/******************************
	 * Access with Access Selector
	 ******************************/
	/**
	 * Simply Access Button with Access Selector
	 */
	public boolean Button(Button btn)
	{
		if (btn == Button.A)
		{
			return getRawButton(1);
		} else if (btn == Button.B)
		{
			return getRawButton(2);
		} else if (btn == Button.X)
		{
			return getRawButton(3);
		} else if (btn == Button.Y)
		{
			return getRawButton(4);
		} else if (btn == Button.LB)
		{
			return getRawButton(5);
		} else if (btn == Button.RB)
		{
			return getRawButton(6);
		} else if (btn == Button.Start)
		{
			return getRawButton(8);
		} else if (btn == Button.Back)
		{
			return getRawButton(7);
		} else if (btn == Button.LT)
		{
			return getRawAxis(2) > 0.4;
		} else if (btn == Button.RT)
		{
			return getRawAxis(3) > 0.4;
		} else
		{
			return false;
		}

	}

	/**
	 * Combined Button Access
	 */
	public boolean Button(int btns)
	{
		if ((btns & Button.A.Value) > 0 && !Button(Button.A))
		{
			return false;
		} else if ((btns & Button.B.Value) > 0 && !Button(Button.B))
		{
			return false;
		} else if ((btns & Button.X.Value) > 0 && !Button(Button.X))
		{
			return false;
		} else if ((btns & Button.Y.Value) > 0 && !Button(Button.Y))
		{
			return false;
		} else if ((btns & Button.LB.Value) > 0 && !Button(Button.LB))
		{
			return false;
		} else if ((btns & Button.RB.Value) > 0 && !Button(Button.RB))
		{
			return false;
		} else if ((btns & Button.LT.Value) > 0 && !Button(Button.LT))
		{
			return false;
		} else if ((btns & Button.RT.Value) > 0 && !Button(Button.RT))
		{
			return false;
		}

		return true;
	}

	/**
	 * Simply Access Axis With Access Selector
	 */
	public double Axis(Axis axis)
	{
		if (axis == Axis.LX)
		{
			return Math.abs(getRawAxis(0)) > db_LX ? getRawAxis(0) : 0.0f;
		} else if (axis == Axis.LY)
		{
			return Math.abs(getRawAxis(1)) > db_LY ? getRawAxis(1) : 0.0f;
		} else if (axis == Axis.LT)
		{
			return Math.abs(getRawAxis(2)) > db_LX ? getRawAxis(2) : 0.0f;
		} else if (axis == Axis.RT)
		{
			return Math.abs(getRawAxis(3)) > db_LX ? getRawAxis(3) : 0.0f;
		} else if (axis == Axis.RX)
		{
			return Math.abs(getRawAxis(4)) > db_LX ? getRawAxis(4) : 0.0f;
		} else if (axis == Axis.RY)
		{
			return Math.abs(getRawAxis(5)) > db_LX ? getRawAxis(5) : 0.0f;
		} else
		{
			return 0.0f;
		}

	}

	/**
	 * Access Button Press This function only return true when a button is newly
	 * pressed Whole the button is pressed, this function returns false
	 */
	public boolean ButtonPressed(Button btn)
	{
		if (btn == Button.A)
			if (Button(btn))
			{
				return (f_A) ? false : (f_A = true);
			} else
			{
				return f_A = false;
			}
		if (btn == Button.B)
			if (Button(btn))
			{
				return (f_B) ? false : (f_B = true);
			} else
			{
				return f_B = false;
			}
		if (btn == Button.X)
			if (Button(btn))
			{
				return (f_X) ? false : (f_X = true);
			} else
			{
				return f_X = false;
			}
		if (btn == Button.Y)
			if (Button(btn))
			{
				return (f_Y) ? false : (f_Y = true);
			} else
			{
				return f_Y = false;
			}
		if (btn == Button.LB)
			if (Button(btn))
			{
				return (f_LB) ? false : (f_LB = true);
			} else
			{
				return f_LB = false;
			}
		if (btn == Button.RB)
			if (Button(btn))
			{
				return (f_RB) ? false : (f_RB = true);
			} else
			{
				return f_RB = false;
			}
		if (btn == Button.Back)
			if (Button(btn))
			{
				return (f_Back) ? false : (f_Back = true);
			} else
			{
				return f_Back = false;
			}
		if (btn == Button.Start)
			if (Button(btn))
			{
				return (f_Start) ? false : (f_Start = true);
			} else
			{
				return f_Start = false;
			}
		if (btn == Button.L3)
			if (Button(btn))
			{
				return (f_L3) ? false : (f_L3 = true);
			} else
			{
				return f_L3 = false;
			}
		if (btn == Button.R3)
			if (Button(btn))
			{
				return (f_R3) ? false : (f_R3 = true);
			} else
			{
				return f_R3 = false;
			}
		if (btn == Button.LT)
			if (Button(btn))
			{
				return (f_LT) ? false : (f_LT = true);
			} else
			{
				return f_LT = false;
			}
		if (btn == Button.RT)
			if (Button(btn))
			{
				return (f_RT) ? false : (f_RT = true);
			} else
			{
				return f_RT = false;
			}
		return false;
	}

	/**
	 * Combined Button Pressed
	 */
	public boolean ButtonPressed(int btns)
	{
		if ((btns & Button.A.Value) > 0 && !ButtonPressed(Button.A))
		{
			return false;
		} else if ((btns & Button.B.Value) > 0 && !ButtonPressed(Button.B))
		{
			return false;
		} else if ((btns & Button.X.Value) > 0 && !ButtonPressed(Button.X))
		{
			return false;
		} else if ((btns & Button.Y.Value) > 0 && !ButtonPressed(Button.Y))
		{
			return false;
		} else if ((btns & Button.LB.Value) > 0 && !ButtonPressed(Button.LB))
		{
			return false;
		} else if ((btns & Button.RB.Value) > 0 && !ButtonPressed(Button.RB))
		{
			return false;
		} else if ((btns & Button.LT.Value) > 0 && !ButtonPressed(Button.LT))
		{
			return false;
		} else if ((btns & Button.RT.Value) > 0 && !ButtonPressed(Button.RT))
		{
			return false;
		}

		return true;

	}

	/******************************
	 * Access Individual
	 ******************************/
	/**
	 * Button
	 */
	public boolean ButtonA()
	{
		return Button(Button.A);
	}

	public boolean ButtonB()
	{
		return Button(Button.B);
	}

	public boolean ButtonX()
	{
		return Button(Button.X);
	}

	public boolean ButtonY()
	{
		return Button(Button.Y);
	}

	public boolean ButtonLB()
	{
		return Button(Button.LB);
	}

	public boolean ButtonRB()
	{
		return Button(Button.RB);
	}

	public boolean ButtonBack()
	{
		return Button(Button.Back);
	}

	public boolean ButtonStart()
	{
		return Button(Button.Start);
	}

	public boolean ButtonL3()
	{
		return Button(Button.L3);
	}

	public boolean ButtonR3()
	{
		return Button(Button.R3);
	}

	public boolean ButtonLT()
	{
		return Button(Button.LT);
	}

	public boolean ButtonRT()
	{
		return Button(Button.RT);
	}

	/**
	 * Button Pressed
	 */
	public boolean ButtonPressedA()
	{
		return ButtonPressed(Button.A);
	}

	public boolean ButtonPressedB()
	{
		return ButtonPressed(Button.B);
	}

	public boolean ButtonPressedX()
	{
		return ButtonPressed(Button.X);
	}

	public boolean ButtonPressedY()
	{
		return ButtonPressed(Button.Y);
	}

	public boolean ButtonPressedLB()
	{
		return ButtonPressed(Button.LB);
	}

	public boolean ButtonPressedRB()
	{
		return ButtonPressed(Button.RB);
	}

	public boolean ButtonPressedStart()
	{
		return ButtonPressed(Button.Start);
	}

	public boolean ButtonPressedBack()
	{
		return ButtonPressed(Button.Back);
	}

	public boolean ButtonPressedL3()
	{
		return ButtonPressed(Button.L3);
	}

	public boolean ButtonPressedR3()
	{
		return ButtonPressed(Button.R3);
	}

	public boolean ButtonPressedLT()
	{
		return ButtonPressed(Button.LT);
	}

	public boolean ButtonPressedRT()
	{
		return ButtonPressed(Button.RT);
	}

	/**
	 * Axis
	 */
	public double AxisLX()
	{
		return Axis(Axis.LX);
	}

	public double AxisLY()
	{
		return Axis(Axis.LY);
	}

	public double AxisRX()
	{
		return Axis(Axis.RX);
	}

	public double AxisRY()
	{
		return Axis(Axis.RY);
	}

	public double AxisLT()
	{
		return Axis(Axis.LT);
	}

	public double AxisRT()
	{
		return Axis(Axis.RT);
	}

}
