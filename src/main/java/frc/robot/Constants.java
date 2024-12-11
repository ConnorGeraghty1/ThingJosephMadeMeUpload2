// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType; //Probably responsible for an error - class "CANSparkLowLevel" is now deprecated
import edu.wpi.first.wpilibj.ADIS16470_IMU;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    // Port numbers for driver and operator gamepads. These correspond with the numbers on the USB
    // tab of the DriverStation
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  
  public static final CANSparkMax m_leftMotor = new CANSparkMax(Constants.leftFrontID, MotorType.kBrushed);
  public static final CANSparkMax m_rightMotor = new CANSparkMax(Constants.rightFrontID, MotorType.kBrushed);

  public static final CANSparkMax m_leftMotor1 = new CANSparkMax(Constants.leftRearID, MotorType.kBrushed);
  public static final CANSparkMax m_rightMotor1 = new CANSparkMax(Constants.rightRearID, MotorType.kBrushed);
  
  public static final int leftRearID = 3;
  public static final int leftFrontID = 4;
  public static final int rightRearID = 2;
  public static final int rightFrontID = 1; 

  public static final ADIS16470_IMU imu = new ADIS16470_IMU();

  public static class DrivetrainConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kLeftRearID = 3;
    public static final int kLeftFrontID = 4;
    public static final int kRightRearID = 2;
    public static final int kRightFrontID = 1;

    // Current limit for drivetrain motors
    public static final int kCurrentLimit = 60;
  }

  public static class LauncherConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kFeederID = 5;
    public static final int kLauncherID = 6;

    // Current limit for launcher and feed wheels
    public static final int kLauncherCurrentLimit = 80;
    public static final int kFeedCurrentLimit = 80;

    // Speeds for wheels when intaking and launching. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kLauncherSpeed = 1;
    public static final double kLaunchFeederSpeed = 1;
    public static final double kIntakeLauncherSpeed = -1;
    public static final double kIntakeFeederSpeed = -.2;

    public static final double kLauncherDelay = 1;
  }
}
