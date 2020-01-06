/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.utilities.Limelight;
import frc.robot.utilities.RollingAverage;

import java.util.function.DoubleFunction;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class Drivetrain extends SubsystemBase 
{
  WPI_TalonSRX frontLeft, backLeft, frontRight, backRight;
	SpeedControllerGroup leftDrive;
	SpeedControllerGroup rightDrive;	
	DifferentialDrive drive;
	public Limelight limelight = new Limelight();
	private RollingAverage limelightAngle = new RollingAverage(5);

	private double quickStopAccumulator = 0.0;
	private double wheelDeadBand = 0.03;
	private double throttleDeadBand = 0.02;
	private static final double SENSITIVITY = 0.90;
  private DoubleFunction<Double> limiter = limiter(-0.9, 0.9);

	public Drivetrain() 
	{
		frontLeft = new WPI_TalonSRX(1);
		backLeft = new WPI_TalonSRX(2);
		frontRight = new WPI_TalonSRX(3);
		backRight= new WPI_TalonSRX(4);
		leftDrive = new SpeedControllerGroup(frontLeft, backLeft);
		rightDrive = new SpeedControllerGroup(frontRight, backRight);
		drive = new DifferentialDrive(leftDrive, rightDrive);
  }

	public void drive(double speed, double turnRate)
	{
		drive.arcadeDrive(speed, turnRate);
	}

	public void cheesyDriveWithJoystick(double throttle, double wheel, boolean quickturn) 
	{
    wheel = handleDeadband(wheel, wheelDeadBand);
    throttle = handleDeadband(throttle, throttleDeadBand);


    double overPower;
    double angularPower;

    wheel = dampen(wheel, 0.5);
    wheel = dampen(wheel, 0.5);
    wheel = dampen(wheel, 0.5);

		if(quickturn)
		{
			if (Math.abs(throttle) < 0.2) 
			{
        double alpha = 0.1;
        quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limiter.apply(wheel) * 2;
      }
      overPower = 1.0;
      angularPower = wheel;
      angularPower *= 0.45;
		} 
		else 
		{
      overPower = 0.0;
      angularPower = Math.abs(throttle) * wheel * SENSITIVITY - quickStopAccumulator;
      angularPower *= 0.8;
			if(quickStopAccumulator > 1) 
			{
        quickStopAccumulator -= 1;
			} 
			else if(quickStopAccumulator < -1) 
			{
        quickStopAccumulator += 1;
			} 
			else
			{
        quickStopAccumulator = 0.0;
      }
    }

    double rightPwm = throttle - (-1 * angularPower);
    double leftPwm = throttle + (-1 *angularPower);
		if(leftPwm > 1.0) 
		{
      rightPwm -= overPower * (leftPwm - 1.0);
      leftPwm = 1.0;
		} 
		else if(rightPwm > 1.0) 
		{
      leftPwm -= overPower * (rightPwm - 1.0);
      rightPwm = 1.0;
		} 
		else if (leftPwm < -1.0) 
		{
      rightPwm += overPower * (-1.0 - leftPwm);
      leftPwm = -1.0;
		} 
		else if (rightPwm < -1.0) 
		{
      leftPwm += overPower * (-1.0 - rightPwm);
      rightPwm = -1.0;
    }

    setLeftRightMotorOutputs(-1 * leftPwm, -1 * (rightPwm));
	}

	public double handleDeadband(double val, double deadband)
	{
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	
	private static double dampen(double wheel, double wheelNonLinearity) 
	{
    double factor = Math.PI * wheelNonLinearity;
    return Math.sin(factor * wheel) / Math.sin(factor);
	}
	
	private DoubleFunction<Double> limiter(double minimum, double maximum) 
	{
		if (maximum < minimum) 
		{
      throw new IllegalArgumentException("The minimum value cannot exceed the maximum value");
    }
		return (double value) -> 
		{
			if (value > maximum) 
			{
        return maximum;
      }
			if (value < minimum) 
			{
        return minimum;
      }
      return value;
    };
  }

	public void setLeftRightMotorOutputs(double left, double right)
	{
		frontLeft.set(left);
		backLeft.set(left);
		frontRight.set(right);
		backRight.set(right);
	}

	public void driveWithTarget(double throttle, double angle) 
	{
    double yawSpeed = 1.25 * angle/ 30; 

    cheesyDriveWithJoystick(-0.2, yawSpeed, false);
  }

	public void driveWithTargetNew()
	{
		if(limelight.hasTarget())
		{
      double angle = limelight.getAngle();
      double area = limelight.getArea();
      
      limelightAngle.add(angle);

      double wheel = 1.25 / 30 * (limelightAngle.getAverage());
      
      cheesyDriveWithJoystick(0, wheel, false);
    }  
  }
	
	public void stop() 
	{
		drive.stopMotor();
	}
}