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
  public static final int intakePivotID = 18;
  public static final int intakeWheelsID = 19;

/* Clearance positions*/
  public static final double crossbarClearancePos = 0; //elevator height to clear crossbar (must be below)
  public static final double intakeEndeffectorClearancePos = 0; //intake pivot position to clear the endeffector (intake must be deployed enough)
  public static final double endeffectorElevatorClearancePos = 0; //Endeffector pivot position to clear elevator
  
  public static class ReefScoringLocations{
      // REEF PRE-SCORING POSES
      public static   final Pose2d  BLUEPRESCORE_1  = new Pose2d( 6.417, 3.826, Rotation2d.fromDegrees(   	0	  )); 
      public static   final Pose2d  BLUEPRESCORE_2  = new Pose2d( 6.418, 4.217, Rotation2d.fromDegrees(   	0	  )); 
      public static   final Pose2d  BLUEPRESCORE_3  = new Pose2d( 5.629, 5.585, Rotation2d.fromDegrees(   	60	));    
      public static   final Pose2d  BLUEPRESCORE_4  = new Pose2d( 5.289, 5.801, Rotation2d.fromDegrees(   	60	));    
      public static   final Pose2d  BLUEPRESCORE_5  = new Pose2d( 3.703, 5.782, Rotation2d.fromDegrees(   	120	));    
      public static   final Pose2d  BLUEPRESCORE_6  = new Pose2d( 3.360, 5.612, Rotation2d.fromDegrees(   	120	));    
      public static   final Pose2d  BLUEPRESCORE_7  = new Pose2d( 2.561, 4.217, Rotation2d.fromDegrees(   	180	)); 
      public static   final Pose2d  BLUEPRESCORE_8  = new Pose2d( 2.562, 3.826, Rotation2d.fromDegrees(   	180	)); 
      public static   final Pose2d  BLUEPRESCORE_9  = new Pose2d( 3.351, 2.451, Rotation2d.fromDegrees(   -120         ));  
      public static   final Pose2d  BLUEPRESCORE_10 = new Pose2d( 3.689, 2.262, Rotation2d.fromDegrees(   -120         ));  
      public static   final Pose2d  BLUEPRESCORE_11 = new Pose2d( 5.283, 2.255, Rotation2d.fromDegrees(   	-60	        )); 
      public static   final Pose2d  BLUEPRESCORE_12 = new Pose2d( 5.620, 2.450, Rotation2d.fromDegrees(   	-60	        )); 

      public	static	final	Pose2d	REDPRESCORE_1	  =	new	Pose2d(	11.118,	4.220,	Rotation2d.fromDegrees( 180 ));   
      public	static	final	Pose2d	REDPRESCORE_2	  =	new	Pose2d(	11.117,	3.829,	Rotation2d.fromDegrees( 180 ));   
      public	static	final	Pose2d	REDPRESCORE_3	  =	new	Pose2d(	11.906,	2.461,	Rotation2d.fromDegrees( -120));	          
      public	static	final	Pose2d	REDPRESCORE_4	  =	new	Pose2d(	12.246,	2.245,	Rotation2d.fromDegrees( -120));	          
      public	static	final	Pose2d	REDPRESCORE_5	  =	new	Pose2d(	13.832,	2.264,	Rotation2d.fromDegrees( -60 ));	          
      public	static	final	Pose2d	REDPRESCORE_6	  =	new	Pose2d(	14.175,	2.434,	Rotation2d.fromDegrees( -60 ));	          
      public	static	final	Pose2d	REDPRESCORE_7	  =	new	Pose2d(	14.974,	3.829,	Rotation2d.fromDegrees( 0   ));   
      public	static	final	Pose2d	REDPRESCORE_8	  =	new	Pose2d(	14.973,	4.220,	Rotation2d.fromDegrees( 0   ));   
      public	static	final	Pose2d	REDPRESCORE_9	  =	new	Pose2d(	14.184,	5.595,	Rotation2d.fromDegrees( 60  ));   
      public	static	final	Pose2d	REDPRESCORE_10	=	new	Pose2d(	13.846,	5.784,	Rotation2d.fromDegrees( 60  ));   
      public	static	final	Pose2d	REDPRESCORE_11	=	new	Pose2d(	12.252,	5.791,	Rotation2d.fromDegrees( 120 ));    
      public	static	final	Pose2d	REDPRESCORE_12	=	new	Pose2d(	11.915,	5.596,	Rotation2d.fromDegrees( 120 ));    

  // REEF SCORING POSES
      public static   final Pose2d  BLUE_1  = new Pose2d( 6.119, 3.857, Rotation2d.fromDegrees(      	0	  )); 
      public static   final Pose2d  BLUE_2  = new Pose2d( 6.119, 4.187, Rotation2d.fromDegrees(      	0	  )); 
      public static   final Pose2d  BLUE_3  = new Pose2d( 5.452, 5.343, Rotation2d.fromDegrees(      	60	)); 
      public static   final Pose2d  BLUE_4  = new Pose2d( 5.166, 5.527, Rotation2d.fromDegrees(      	60	)); 
      public static   final Pose2d  BLUE_5  = new Pose2d( 3.826, 5.508, Rotation2d.fromDegrees(      	120	)); 
      public static   final Pose2d  BLUE_6  = new Pose2d( 3.534, 5.368, Rotation2d.fromDegrees(      	120	)); 
      public static   final Pose2d  BLUE_7  = new Pose2d( 2.860, 4.187, Rotation2d.fromDegrees(      	180	)); 
      public static   final Pose2d  BLUE_8  = new Pose2d( 2.860, 3.857, Rotation2d.fromDegrees(      	180	)); 
      public static   final Pose2d  BLUE_9  = new Pose2d( 3.527, 2.694, Rotation2d.fromDegrees(      -120         ));  
      public static   final Pose2d  BLUE_10 = new Pose2d( 3.813, 2.535, Rotation2d.fromDegrees(      -120         ));  
      public static   final Pose2d  BLUE_11 = new Pose2d( 5.160, 2.529, Rotation2d.fromDegrees(      	-60	        )); 
      public static   final Pose2d  BLUE_12 = new Pose2d( 5.445, 2.694, Rotation2d.fromDegrees(      	-60	        )); 

      public	static	final	Pose2d	RED_1	  =	new	Pose2d(	11.416	,	4.189	,	Rotation2d.fromDegrees(	 180 ));   
      public	static	final	Pose2d	RED_2	  =	new	Pose2d(	11.416	,	3.859	,	Rotation2d.fromDegrees(	 180 ));   
      public	static	final	Pose2d	RED_3	  =	new	Pose2d(	12.083	,	2.703	,	Rotation2d.fromDegrees(	 -120));	          
      public	static	final	Pose2d	RED_4	  =	new	Pose2d(	12.369	,	2.519	,	Rotation2d.fromDegrees(	 -120));	          
      public	static	final	Pose2d	RED_5	  =	new	Pose2d(	13.709	,	2.538	,	Rotation2d.fromDegrees(	 -60 ));	          
      public	static	final	Pose2d	RED_6	  =	new	Pose2d(	14.001	,	2.678	,	Rotation2d.fromDegrees(	 -60 ));	          
      public	static	final	Pose2d	RED_7	  =	new	Pose2d(	14.675	,	3.859	,	Rotation2d.fromDegrees(	 0   ));   
      public	static	final	Pose2d	RED_8	  =	new	Pose2d(	14.675	,	4.189	,	Rotation2d.fromDegrees(	 0   ));   
      public	static	final	Pose2d	RED_9	  =	new	Pose2d(	14.008	,	5.352	,	Rotation2d.fromDegrees(  60  ));   
      public	static	final	Pose2d	RED_10	=	new	Pose2d(	13.722	,	5.511	,	Rotation2d.fromDegrees(  60  ));   
      public	static	final	Pose2d	RED_11	=	new	Pose2d(	12.375	,	5.517	,	Rotation2d.fromDegrees(	 120 ));   
      public	static	final	Pose2d	RED_12	=	new	Pose2d(	12.090	,	5.352	,	Rotation2d.fromDegrees(	 120 ));   
      
  }
}
