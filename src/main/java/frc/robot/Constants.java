// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (136.753) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  
  /* EndEffector and Elevator IDs */
  public static final int endEffectorWheelID = 14;
  public static final int endEffectorPivotID = 15;
  public static final int elevatorLeftID = 17;
  public static final int elevatorRightID = 16;

  public static class ReefScoringLocations{
  // BRANCH POSES
      public static final Pose2d BLUE_1 = new Pose2d(2.860, 4.187, Rotation2d.fromDegrees(0));
      public static final Pose2d BLUE_2 = new Pose2d(2.860, 3.857, Rotation2d.fromDegrees(0));
      public static final Pose2d BLUE_3 = new Pose2d(3.527, 2.694, Rotation2d.fromDegrees(60));
      public static final Pose2d BLUE_4 = new Pose2d(3.813, 2.535, Rotation2d.fromDegrees(60));
      public static final Pose2d BLUE_5 = new Pose2d(5.160, 2.529, Rotation2d.fromDegrees(120));
      public static final Pose2d BLUE_6 = new Pose2d(5.445, 2.694, Rotation2d.fromDegrees(120));
      public static final Pose2d BLUE_7 = new Pose2d(6.119, 3.857, Rotation2d.fromDegrees(180));
      public static final Pose2d BLUE_8 = new Pose2d(6.119, 4.187, Rotation2d.fromDegrees(180));
      public static final Pose2d BLUE_9 = new Pose2d(5.452, 5.343, Rotation2d.fromDegrees(-120));
      public static final Pose2d BLUE_10 = new Pose2d(5.166, 5.527, Rotation2d.fromDegrees(-120));
      public static final Pose2d BLUE_11 = new Pose2d(3.826, 5.508, Rotation2d.fromDegrees(-60));
      public static final Pose2d BLUE_12 = new Pose2d(3.534, 5.368, Rotation2d.fromDegrees(-60));
  }
}
