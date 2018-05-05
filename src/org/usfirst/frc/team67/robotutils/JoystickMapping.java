package org.usfirst.frc.team67.robotutils;

public class JoystickMapping
{
	private Point2D[] map;

	public JoystickMapping(Point2D mapping[], double deadLow, double deadHigh)
	{
		assert (mapping.length == 8);
		map = mapping;
		deadBandHigh = deadHigh;
		deadBandLow = deadLow;
	}

	public double GetMappedValue(double input)
	{
		double output;
		if (deadBandHigh >= input || deadBandLow <= input)
		{
			if (input > map[0].x && input < map[1].x)
			{
				output = interpolate(input, map[0].x, map[0].y, map[1].x, map[1].y);
			} else if (input >= map[1].x && input < map[2].x)
			{
				output = interpolate(input, map[1].x, map[1].y, map[2].x, map[2].y);
			} else if (input >= map[2].x && input < map[3].x)
			{
				output = interpolate(input, map[2].x, map[2].y, map[3].x, map[3].y);
			} else if (input >= map[3].x && input < map[4].x)
			{
				output = interpolate(input, map[3].x, map[3].y, map[4].x, map[4].y);
			} else if (input >= map[4].x && input < map[5].x)
			{
				output = interpolate(input, map[4].x, map[4].y, map[5].x, map[5].y);
			} else if (input >= map[5].x && input < map[6].x)
			{
				output = interpolate(input, map[5].x, map[5].y, map[6].x, map[6].y);
			} else if (input >= map[6].x && input < map[7].x)
			{
				output = interpolate(input, map[6].x, map[6].y, map[7].x, map[7].y);
			} else if (input >= map[7].x)
			{
				output = map[7].y;
			} else if (input <= map[0].x)
			{
				output = map[0].y;
			} else
			{
				output = 0;
				System.out.println("Falling Through !!!!!!\n");
			}
		} else
		{
			output = 0;
		}

		return output;

	}

	private double interpolate(double xInput, double x1, double y1, double x2, double y2)
	{
		if (x2 != x1)
			return (y2 - y1) / (x2 - x1) * (xInput - x1) + y1;
		else
		{
			System.out.print("Error Don't Divide By ZERO!!!!!!\n");
			return 0;
		}
	}

	private double deadBandHigh;
	private double deadBandLow;
}
