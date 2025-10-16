// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

  public static final double scoringPositionTolerance = 0.2; // Scoring position tolerance for superstructure
  public static final double positionToleranceUp = 0.2; // general position tolerance for superstructure
  public static final double positionTolerance = 0.5; // general position tolerance for superstructure

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
  public static final int eeWheelsLeftID = 14;
  public static final int eeWheelsRightID = 19;
  public static final int endEffectorPivotID = 15;
  public static final int elevatorLeftID = 17;
  public static final int elevatorRightID = 16;
  public static final int armID = 18;//arm

  public static final int topFunnelWheelsID = 21;
  
  public static final int bottomFunnelWheelsID = 20;
  public static final int climbPivotID = 22;


/* Clearance positions*/
  public static final double intakePivotGearRatio = 270; // 200:1 gear ratio
  public static final double endEffectorPivotGearRatio = 60 / 8 * 4; // 30:1 gear ratio
  public static final double crossbarClearancePos = 20; //elevator height to clear crossbar (must be below)
  public static final double intakeEndeffectorClearancePos = 3; //intake pivot position to clear the endeffector (intake must be deployed enough)
  public static final double endeffectorElevatorClearancePos = 10 * endEffectorPivotGearRatio / 360 / 2.25; //Endeffector pivot position to clear elevator
  
  

  public static class ReefScoringLocations{
    
      // REEF PRE-SCORING POSES
      public static   final Pose2d  BLUEALGAE_12        = new Pose2d( 6.225,	4.0,   Rotation2d.fromDegrees( 	0+180 	)); //
      public static   final Pose2d  BLUEALGAE_11        = BLUEALGAE_12; //
      public static   final Pose2d  BLUEALGAE_10        = new Pose2d( 5.36,	5.53,   Rotation2d.fromDegrees(  60+180          ));  //
      public static   final Pose2d  BLUEALGAE_9	       = BLUEALGAE_10;  //
      public static   final Pose2d  BLUEALGAE_8	       = new Pose2d( 3.62, 5.53,   Rotation2d.fromDegrees(  	120+180	         )); //
      public static   final Pose2d  BLUEALGAE_7	       = BLUEALGAE_8;  //
      public static   final Pose2d  BLUEALGAE_6	       = new Pose2d( 2.748, 4.0,    Rotation2d.fromDegrees( 	0	   )); //
      public static   final Pose2d  BLUEALGAE_5	       = BLUEALGAE_6; //
      public static   final Pose2d  BLUEALGAE_4	       = new Pose2d( 3.62, 2.52,   Rotation2d.fromDegrees( 	-120+180 	)); //
      public static   final Pose2d  BLUEALGAE_3	       = BLUEALGAE_4; //
      public static   final Pose2d  BLUEALGAE_2	       = new Pose2d( 5.36, 2.52,   Rotation2d.fromDegrees( 	-60+180 	)); //
      public static   final Pose2d  BLUEALGAE_1	       = BLUEALGAE_2; //

      public	static	final	Pose2d	REDALGAE_12	      =	new	Pose2d(	11.317,	4.0,	Rotation2d.fromDegrees(	 0    )); //
      public	static	final	Pose2d	REDALGAE_11	      =	REDALGAE_12; //
      public	static	final	Pose2d	REDALGAE_10	      =	new	Pose2d(	12.17,	2.52,	Rotation2d.fromDegrees(	 -120+180   )); //
      public	static	final	Pose2d	REDALGAE_9	        =	REDALGAE_10; //
      public	static	final	Pose2d	REDALGAE_8	        =	new	Pose2d(	13.927,	2.52,	Rotation2d.fromDegrees(	 -60+180  )); //
      public	static	final	Pose2d	REDALGAE_7	        =	REDALGAE_8; //
      public	static	final	Pose2d	REDALGAE_6	        =	new	Pose2d(	14.792,	4.0,	Rotation2d.fromDegrees(	 0+180  )); //
      public	static	final	Pose2d	REDALGAE_5	        =	REDALGAE_6; //
      public	static	final	Pose2d	REDALGAE_4	        =	new	Pose2d(	13.936,	5.53,	Rotation2d.fromDegrees(  60+180 ));	//
      public	static	final	Pose2d	REDALGAE_3	        =	REDALGAE_4;	//
      public	static	final	Pose2d	REDALGAE_2	        =	new	Pose2d(	12.189,	5.53,	Rotation2d.fromDegrees(	 120+180  ));	//
      public	static	final	Pose2d	REDALGAE_1	        =	REDALGAE_2; //
            // REEF PRE-SCORING POSES
      public static   final Pose2d  BLUEPRESCORE_12  = new Pose2d( 6.417, 3.826, Rotation2d.fromDegrees(   	0	 +180 )); 
      public static   final Pose2d  BLUEPRESCORE_11  = new Pose2d( 6.418, 4.217, Rotation2d.fromDegrees(   	0	+180  )); 
      public static   final Pose2d  BLUEPRESCORE_10  = new Pose2d( 5.629, 5.585, Rotation2d.fromDegrees(   	60-180	));    
      public static   final Pose2d  BLUEPRESCORE_9  = new Pose2d( 5.289, 5.801, Rotation2d.fromDegrees(   	60-180	));    
      public static   final Pose2d  BLUEPRESCORE_8  = new Pose2d( 3.703, 5.782, Rotation2d.fromDegrees(   	120-180	));    
      public static   final Pose2d  BLUEPRESCORE_7  = new Pose2d( 3.360, 5.612, Rotation2d.fromDegrees(   	120-180	));    
      public static   final Pose2d  BLUEPRESCORE_6  = new Pose2d( 2.561, 4.217, Rotation2d.fromDegrees(   	0	)); 
      public static   final Pose2d  BLUEPRESCORE_5  = new Pose2d( 2.562, 3.826, Rotation2d.fromDegrees(   	0)); 
      public static   final Pose2d  BLUEPRESCORE_4  = new Pose2d( 3.351, 2.451, Rotation2d.fromDegrees(   -120      +180   ));  
      public static   final Pose2d  BLUEPRESCORE_3 = new Pose2d( 3.689, 2.262, Rotation2d.fromDegrees(   -120        +180 ));  
      public static   final Pose2d  BLUEPRESCORE_2 = new Pose2d( 5.283, 2.255, Rotation2d.fromDegrees(   	-60	      +180  )); 
      public static   final Pose2d  BLUEPRESCORE_1 = new Pose2d( 5.620, 2.450, Rotation2d.fromDegrees(   	-60	       +180 )); 

      public	static	final	Pose2d	REDPRESCORE_12	  =	new	Pose2d(	11.118,	4.220,	Rotation2d.fromDegrees( 180 ));   
      public	static	final	Pose2d	REDPRESCORE_11	  =	new	Pose2d(	11.117,	3.829,	Rotation2d.fromDegrees( 180 ));   
      public	static	final	Pose2d	REDPRESCORE_10	  =	new	Pose2d(	11.906,	2.461,	Rotation2d.fromDegrees( -120));	          
      public	static	final	Pose2d	REDPRESCORE_9	  =	new	Pose2d(	12.246,	2.245,	Rotation2d.fromDegrees( -120));	          
      public	static	final	Pose2d	REDPRESCORE_8	  =	new	Pose2d(	13.832,	2.264,	Rotation2d.fromDegrees( -60 ));	          
      public	static	final	Pose2d	REDPRESCORE_7	  =	new	Pose2d(	14.175,	2.434,	Rotation2d.fromDegrees( -60 ));	          
      public	static	final	Pose2d	REDPRESCORE_6	  =	new	Pose2d(	14.974,	3.829,	Rotation2d.fromDegrees( 0   ));   
      public	static	final	Pose2d	REDPRESCORE_5	  =	new	Pose2d(	14.973,	4.220,	Rotation2d.fromDegrees( 0   ));   
      public	static	final	Pose2d	REDPRESCORE_4	  =	new	Pose2d(	14.184,	5.595,	Rotation2d.fromDegrees( 60  ));   
      public	static	final	Pose2d	REDPRESCORE_3	=	new	Pose2d(	13.846,	5.784,	Rotation2d.fromDegrees( 60  ));   
      public	static	final	Pose2d	REDPRESCORE_2	=	new	Pose2d(	12.252,	5.791,	Rotation2d.fromDegrees( 120 ));    
      public	static	final	Pose2d	REDPRESCORE_1	=	new	Pose2d(	11.915,	5.596,	Rotation2d.fromDegrees( 120 ));    

  // REEF SCORING POSES - Contra Measure
      public static   final Pose2d  BLUE_12        = new Pose2d( 6.193,	3.855,   Rotation2d.fromDegrees( 	-0.3+180	)); // /* */
      public static   final Pose2d  BLUE_11        = new Pose2d( 6.195,	4.232,   Rotation2d.fromDegrees( 	2.5+180 	)); // /* */
      public static   final Pose2d  BLUE_10        = new Pose2d( 5.468,	5.430,   Rotation2d.fromDegrees(  60.5+180         ));  //
      public static   final Pose2d  BLUE_9	       = new Pose2d( 5.144,	5.617,   Rotation2d.fromDegrees(  62.7 +180         ));  // /* */
      public static   final Pose2d  BLUE_8	       = new Pose2d( 3.769, 5.580,   Rotation2d.fromDegrees(  	120.8+180	         )); // /* */
      public static   final Pose2d  BLUE_7	       = new Pose2d( 3.452, 5.397,   Rotation2d.fromDegrees(  	121.7+180	         ));  // /* */
      public static   final Pose2d  BLUE_6	       = new Pose2d( 2.784, 4.156,    Rotation2d.fromDegrees( 	-179.2+180	   )); // /* */
      public static   final Pose2d  BLUE_5	       = new Pose2d( 2.785, 3.813,    Rotation2d.fromDegrees( 	-178+180 	  )); // /* */
      public static   final Pose2d  BLUE_4	       = new Pose2d( 3.536, 2.604,   Rotation2d.fromDegrees( 	-118.8+180 	)); // /* */
      public static   final Pose2d  BLUE_3	       = new Pose2d( 3.847, 2.432,   Rotation2d.fromDegrees( 	-118.8+180 	)); // /* */
      public static   final Pose2d  BLUE_2	       = new Pose2d( 5.205, 2.471,   Rotation2d.fromDegrees( 	-60+180 	)); // /* */
      public static   final Pose2d  BLUE_1	       = new Pose2d( 5.52, 2.65,   Rotation2d.fromDegrees( 	-60+180 	)); // /* */
      public	static	final	Pose2d	BLUEBARGE	     = new Pose2d(	6.5, 6.0,	Rotation2d.fromDegrees( 	-40 	)); // /*note: super approx. value! Also made it turn a bit to prevent bounce-outs */
      public	static	final	Pose2d	BLUEBARGEOPP   = new Pose2d(	9.5, 6.0,	Rotation2d.fromDegrees( 	180 -40 	)); // /*note: super approx. value! Also made it turn a bit to prevent bounce-outs */
      public	static	final	Pose2d	BLUEPROCESSOR	 = new Pose2d(	6.5, 0.2,	Rotation2d.fromDegrees( 	90 	)); // /*note: super approx. value! */

      public	static	final	Pose2d	RED_12	      =	new	Pose2d(	11.351,	4.185,	Rotation2d.fromDegrees(	 -179.3 +180   )); // /* */
      public	static	final	Pose2d	RED_11	      =	new	Pose2d(	11.352,	3.838,	Rotation2d.fromDegrees(	 -178+180    )); // /* */
      public	static	final	Pose2d	RED_10	      =	new	Pose2d(	12.082,	2.622,	Rotation2d.fromDegrees(	 -120.05 +180  )); //
      public	static	final	Pose2d	RED_9	        =	new	Pose2d(	12.396,	2.441,	Rotation2d.fromDegrees(	 -118.1+180   )); //
      public	static	final	Pose2d	RED_8	        =	new	Pose2d(	13.775,	2.470,	Rotation2d.fromDegrees(	 -59.8+180  )); // /* */
      public	static	final	Pose2d	RED_7	        =	new	Pose2d(	14.098,	2.655,	Rotation2d.fromDegrees(	 -57.4 +180 )); // /* */
      public	static	final	Pose2d	RED_6	        =	new	Pose2d(	14.765,	3.870,	Rotation2d.fromDegrees(	 -.05+180  )); // /* */
      public	static	final	Pose2d	RED_5	        =	new	Pose2d(	14.764,	4.250,	Rotation2d.fromDegrees(	 2.71+180  )); //
      public	static	final	Pose2d	RED_4	        =	new	Pose2d(	14.053,	5.421,	Rotation2d.fromDegrees(  59.7+180 ));	// /* */
      public	static	final	Pose2d	RED_3	        =	new	Pose2d(	13.695,	5.627,	Rotation2d.fromDegrees(  62.9+180 ));	// /* */
      public	static	final	Pose2d	RED_2	        =	new	Pose2d(	12.339,	5.581,	Rotation2d.fromDegrees(	 120.08+180  ));	// 
      public	static	final	Pose2d	RED_1	        =	new	Pose2d(	12.011,	5.388,	Rotation2d.fromDegrees(	 122.8+180  )); // /* */
      public	static	final	Pose2d	REDBARGE	    =	new	Pose2d(	9.5,	2.5,	Rotation2d.fromDegrees(	 180-40  )); // /*note: super approx. value! Also made it turn a bit to prevent bounce-outs */
      public	static	final	Pose2d	REDBARGEOPP   =	new	Pose2d(	6.5,	2.5,	Rotation2d.fromDegrees(	 -40  )); // /*note: super approx. value! Also made it turn a bit to prevent bounce-outs */
      public	static	final	Pose2d	REDPROCESSOR  =	new	Pose2d(	11.4,	7.9,	Rotation2d.fromDegrees(	 90  )); // /*note: super approx. value! */
      
      
      public static Pose2d getScorePose(Boolean isRedAlliance, Double selectPose){
        Pose2d scoreDrivePose = new Pose2d(); 
        if(isRedAlliance){
          if(selectPose ==  1 ) scoreDrivePose = RED_1;
          if(selectPose ==  2 ) scoreDrivePose = RED_2;
          if(selectPose ==  3 ) scoreDrivePose = RED_3;
          if(selectPose ==  4 ) scoreDrivePose = RED_4;
          if(selectPose ==  5 ) scoreDrivePose = RED_5;
          if(selectPose ==  6 ) scoreDrivePose = RED_6;
          if(selectPose ==  7 ) scoreDrivePose = RED_7;
          if(selectPose ==  8 ) scoreDrivePose = RED_8;
          if(selectPose ==  9 ) scoreDrivePose = RED_9;
          if(selectPose ==  10 ) scoreDrivePose= RED_10;
          if(selectPose ==  11 ) scoreDrivePose= RED_11;
          if(selectPose ==  12 ) scoreDrivePose= RED_12;
      }else{
          if(selectPose ==  1 ) scoreDrivePose = BLUE_1;
          if(selectPose ==  2 ) scoreDrivePose = BLUE_2;
          if(selectPose ==  3 ) scoreDrivePose = BLUE_3;
          if(selectPose ==  4 ) scoreDrivePose = BLUE_4;
          if(selectPose ==  5 ) scoreDrivePose = BLUE_5;
          if(selectPose ==  6 ) scoreDrivePose = BLUE_6;
          if(selectPose ==  7 ) scoreDrivePose = BLUE_7;
          if(selectPose ==  8 ) scoreDrivePose = BLUE_8;
          if(selectPose ==  9 ) scoreDrivePose = BLUE_9;
          if(selectPose ==  10 ) scoreDrivePose= BLUE_10;
          if(selectPose ==  11 ) scoreDrivePose= BLUE_11;
          if(selectPose ==  12 ) scoreDrivePose= BLUE_12;
      }
      return scoreDrivePose;
      }

      public static Pose2d getAlgaeGrabPose(Boolean isRedAlliance, Double selectPose){
        Pose2d scoreDrivePose = new Pose2d(); 
        if(isRedAlliance){
          if(selectPose ==  1 ) scoreDrivePose = REDALGAE_1;
          if(selectPose ==  2 ) scoreDrivePose = REDALGAE_2;
          if(selectPose ==  3 ) scoreDrivePose = REDALGAE_3;
          if(selectPose ==  4 ) scoreDrivePose = REDALGAE_4;
          if(selectPose ==  5 ) scoreDrivePose = REDALGAE_5;
          if(selectPose ==  6 ) scoreDrivePose = REDALGAE_6;
          if(selectPose ==  7 ) scoreDrivePose = REDALGAE_7;
          if(selectPose ==  8 ) scoreDrivePose = REDALGAE_8;
          if(selectPose ==  9 ) scoreDrivePose = REDALGAE_9;
          if(selectPose ==  10 ) scoreDrivePose= REDALGAE_10;
          if(selectPose ==  11 ) scoreDrivePose= REDALGAE_11;
          if(selectPose ==  12 ) scoreDrivePose= REDALGAE_12;
      }else{
          if(selectPose ==  1 ) scoreDrivePose = BLUEALGAE_1;
          if(selectPose ==  2 ) scoreDrivePose = BLUEALGAE_2;
          if(selectPose ==  3 ) scoreDrivePose = BLUEALGAE_3;
          if(selectPose ==  4 ) scoreDrivePose = BLUEALGAE_4;
          if(selectPose ==  5 ) scoreDrivePose = BLUEALGAE_5;
          if(selectPose ==  6 ) scoreDrivePose = BLUEALGAE_6;
          if(selectPose ==  7 ) scoreDrivePose = BLUEALGAE_7;
          if(selectPose ==  8 ) scoreDrivePose = BLUEALGAE_8;
          if(selectPose ==  9 ) scoreDrivePose = BLUEALGAE_9;
          if(selectPose ==  10 ) scoreDrivePose= BLUEALGAE_10;
          if(selectPose ==  11 ) scoreDrivePose= BLUEALGAE_11;
          if(selectPose ==  12 ) scoreDrivePose= BLUEALGAE_12;
      }
      return scoreDrivePose;
      }

      public static Pose2d getPrescorePose(Boolean isRedAlliance, Double selectPose){
        Pose2d prescoreDrivePose = new Pose2d();
        if(isRedAlliance){
          if(selectPose ==  1 ) prescoreDrivePose = REDPRESCORE_1;
          if(selectPose ==  2 ) prescoreDrivePose = REDPRESCORE_2;
          if(selectPose ==  3 ) prescoreDrivePose = REDPRESCORE_3;
          if(selectPose ==  4 ) prescoreDrivePose = REDPRESCORE_4;
          if(selectPose ==  5 ) prescoreDrivePose = REDPRESCORE_5;
          if(selectPose ==  6 ) prescoreDrivePose = REDPRESCORE_6;
          if(selectPose ==  7 ) prescoreDrivePose = REDPRESCORE_7;
          if(selectPose ==  8 ) prescoreDrivePose = REDPRESCORE_8;
          if(selectPose ==  9 ) prescoreDrivePose = REDPRESCORE_9;
          if(selectPose ==  10 ) prescoreDrivePose= REDPRESCORE_10;
          if(selectPose ==  11 ) prescoreDrivePose= REDPRESCORE_11;
          if(selectPose ==  12 ) prescoreDrivePose= REDPRESCORE_12;
          if(selectPose ==  13 ) prescoreDrivePose= REDBARGE;
          if(selectPose ==  14 ) prescoreDrivePose= REDPROCESSOR;
          if(selectPose ==  15 ) prescoreDrivePose= REDBARGEOPP;
          
          
        }else{
          if(selectPose ==  1 ) prescoreDrivePose = BLUEPRESCORE_1;
          if(selectPose ==  2 ) prescoreDrivePose = BLUEPRESCORE_2;
          if(selectPose ==  3 ) prescoreDrivePose = BLUEPRESCORE_3;
          if(selectPose ==  4 ) prescoreDrivePose = BLUEPRESCORE_4;
          if(selectPose ==  5 ) prescoreDrivePose = BLUEPRESCORE_5;
          if(selectPose ==  6 ) prescoreDrivePose = BLUEPRESCORE_6;
          if(selectPose ==  7 ) prescoreDrivePose = BLUEPRESCORE_7;
          if(selectPose ==  8 ) prescoreDrivePose = BLUEPRESCORE_8;
          if(selectPose ==  9 ) prescoreDrivePose = BLUEPRESCORE_9;
          if(selectPose ==  10 ) prescoreDrivePose= BLUEPRESCORE_10;
          if(selectPose ==  11 ) prescoreDrivePose= BLUEPRESCORE_11;
          if(selectPose ==  12 ) prescoreDrivePose= BLUEPRESCORE_12;
          if(selectPose ==  13 ) prescoreDrivePose= BLUEBARGE;
          if(selectPose ==  14 ) prescoreDrivePose= BLUEPROCESSOR;
          if(selectPose ==  15 ) prescoreDrivePose= BLUEBARGEOPP;
    }
    return prescoreDrivePose;
      }
  }
  

  public static class SuperstructureConfigs{
    
        public static TalonFXConfiguration getElevatorConfigLeft() { // ELEVATOR
          TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration();
          elevatorConfigLeft.Slot0.kG = 0.35 * 2.25; // Volts to overcome gravity
          elevatorConfigLeft.Slot0.kS = 0.0; // Volts to overcome static friction
          elevatorConfigLeft.Slot0.kV = 0.12; // Volts for a velocity target of 1 rps
          elevatorConfigLeft.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          elevatorConfigLeft.Slot0.kP = 15;//30 was too high
          elevatorConfigLeft.Slot0.kI = 0.0000001; //
          elevatorConfigLeft.Slot0.kD = 0.02; //
          
          elevatorConfigLeft.CurrentLimits.SupplyCurrentLimit = 40;//
          elevatorConfigLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = .01;
          elevatorConfigLeft.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .01;
          elevatorConfigLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .01;
          elevatorConfigLeft.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .01;

          elevatorConfigLeft.MotionMagic.MotionMagicCruiseVelocity = 80; //100 was smooth, 200 is faster than kraken max 100
          elevatorConfigLeft.MotionMagic.MotionMagicAcceleration = 150+100;// 80 was good, 150 zippy

          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 26.8;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
          elevatorConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          elevatorConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return elevatorConfigLeft;
        }
        public static TalonFXConfiguration getElevatorConfigLeftFast() { // ELEVATOR
          TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration();
          elevatorConfigLeft.Slot0.kG = 0.35 * 2.25; // Volts to overcome gravity
          elevatorConfigLeft.Slot0.kS = 0.0; // Volts to overcome static friction
          elevatorConfigLeft.Slot0.kV = 0.12; // Volts for a velocity target of 1 rps
          elevatorConfigLeft.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          elevatorConfigLeft.Slot0.kP = 15;//30 was too high
          elevatorConfigLeft.Slot0.kI = 0.0000001; //
          elevatorConfigLeft.Slot0.kD = 0.02; //
          
          elevatorConfigLeft.CurrentLimits.SupplyCurrentLimit = 40;//
          elevatorConfigLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = .01;
          elevatorConfigLeft.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .01;
          elevatorConfigLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .01;
          elevatorConfigLeft.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .01;

          elevatorConfigLeft.MotionMagic.MotionMagicCruiseVelocity = 90; //100 was smooth, 200 is faster than kraken max 100
          elevatorConfigLeft.MotionMagic.MotionMagicAcceleration = 200+200;// 80 was good, 150 zippy

          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 26.8;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
          elevatorConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          elevatorConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return elevatorConfigLeft;
        }
        public static TalonFXConfiguration getElevatorConfigRight() {
          TalonFXConfiguration elevatorConfigRight = new TalonFXConfiguration();
          elevatorConfigRight = getElevatorConfigLeft();

          elevatorConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //changed from CounterClockwise_Positive
          return elevatorConfigRight;
        }

        public static TalonFXConfiguration getEndeffectorPivotConfig(){
          TalonFXConfiguration endeffectorPivotConfig = new TalonFXConfiguration();
          endeffectorPivotConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorPivotConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorPivotConfig.Slot0.kV = 0.12; //
          endeffectorPivotConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          endeffectorPivotConfig.Slot0.kP = 15;//
          endeffectorPivotConfig.Slot0.kI = 0.00001; //
          endeffectorPivotConfig.Slot0.kD = 0.25; //

          endeffectorPivotConfig.CurrentLimits.SupplyCurrentLimit = 15;//
          endeffectorPivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          endeffectorPivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          endeffectorPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .04;
          endeffectorPivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          endeffectorPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 80; //20 was smooth
          endeffectorPivotConfig.MotionMagic.MotionMagicAcceleration = 200+200; //50 was smooth
          endeffectorPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          /*endeffectorPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          endeffectorPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          endeffectorPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20;
          endeffectorPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -10;*/
          endeffectorPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return endeffectorPivotConfig;
        }
        public static TalonFXConfiguration getEndeffectorPivotConfigFast(){
          TalonFXConfiguration endeffectorPivotConfig = new TalonFXConfiguration();
          endeffectorPivotConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorPivotConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorPivotConfig.Slot0.kV = 0.12; //
          endeffectorPivotConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          endeffectorPivotConfig.Slot0.kP = 15;//
          endeffectorPivotConfig.Slot0.kI = 0.00001; //
          endeffectorPivotConfig.Slot0.kD = 0.25; //

          endeffectorPivotConfig.CurrentLimits.SupplyCurrentLimit = 15;//
          endeffectorPivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          endeffectorPivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          endeffectorPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .04;
          endeffectorPivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          endeffectorPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 100; //20 was smooth
          endeffectorPivotConfig.MotionMagic.MotionMagicAcceleration = 200; //50 was smooth
          endeffectorPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          /*endeffectorPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          endeffectorPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          endeffectorPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20;
          endeffectorPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -10;*/
          endeffectorPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return endeffectorPivotConfig;
        }

        public static TalonFXConfiguration getIntakeWheelsConfiguration(){
          TalonFXConfiguration intakeWheelsConfig = new TalonFXConfiguration();
          intakeWheelsConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          intakeWheelsConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          //intakeWheelsConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
          intakeWheelsConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          intakeWheelsConfig.Slot0.kP = 30;//
          intakeWheelsConfig.Slot0.kI = 0.0001; //
          intakeWheelsConfig.Slot0.kV = 0.12; //
          intakeWheelsConfig.Slot0.kD = 0.00001; //
          intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 15;//
          intakeWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
          intakeWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
          intakeWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
          intakeWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
          intakeWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 35; //stolen from 3255
          intakeWheelsConfig.MotionMagic.MotionMagicAcceleration = 150; //also stolen from 3255
          return intakeWheelsConfig;
        }

        public static TalonFXConfiguration getEndeffectorWheelsConfiguration(){
          TalonFXConfiguration endeffectorWheelsConfig = new TalonFXConfiguration();
          endeffectorWheelsConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorWheelsConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorWheelsConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          endeffectorWheelsConfig.Slot0.kP = 10;//
//          endeffectorWheelsConfig.Slot0.kI = 0.01; //
          endeffectorWheelsConfig.Slot0.kV = 0.12; //
          endeffectorWheelsConfig.Slot0.kD = 0.2; //
          endeffectorWheelsConfig.CurrentLimits.SupplyCurrentLimit = 15;//
          endeffectorWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          endeffectorWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          endeffectorWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          endeffectorWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          endeffectorWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 50; //
          endeffectorWheelsConfig.MotionMagic.MotionMagicAcceleration =  200; //
          endeffectorWheelsConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          return endeffectorWheelsConfig;
        }

        public static TalonFXConfiguration getEndeffectorWheelsConfigurationLeft(){
          TalonFXConfiguration endeffectorWheelsConfig = new TalonFXConfiguration();
          endeffectorWheelsConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorWheelsConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorWheelsConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          endeffectorWheelsConfig.Slot0.kP = 10;//
//          endeffectorWheelsConfig.Slot0.kI = 0.01; //
          endeffectorWheelsConfig.Slot0.kV = 0.12; //
          endeffectorWheelsConfig.Slot0.kD = 0.2; //
          endeffectorWheelsConfig.CurrentLimits.SupplyCurrentLimit = 15;//
          endeffectorWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          endeffectorWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          endeffectorWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          endeffectorWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          endeffectorWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 50; //
          endeffectorWheelsConfig.MotionMagic.MotionMagicAcceleration =  200; //
          endeffectorWheelsConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return endeffectorWheelsConfig;
        }

        public static TalonFXConfiguration getFunnelWheelsConfiguration(){
          TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
          funnelConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          funnelConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          funnelConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          funnelConfig.Slot0.kV = 0.12; //
          funnelConfig.Slot0.kP = 20;//
          funnelConfig.Slot0.kI = 0.00; //
          funnelConfig.Slot0.kD = 0.00001; //
          funnelConfig.CurrentLimits.SupplyCurrentLimit = 15;//was 20
          funnelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
          funnelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
          funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
          funnelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
          funnelConfig.MotionMagic.MotionMagicCruiseVelocity = 100; //stolen from 3255, added '/10' to start slow
          funnelConfig.MotionMagic.MotionMagicAcceleration = 100; //also stolen from 3255, added '/10' to start slow
          funnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return funnelConfig;
        }

        public static TalonFXConfiguration getClimbPivotConfiguration(){
          TalonFXConfiguration intakePivotLeftConfig = new TalonFXConfiguration();
          intakePivotLeftConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          intakePivotLeftConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          intakePivotLeftConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          intakePivotLeftConfig.Slot0.kP = 5;// was 8
          intakePivotLeftConfig.Slot0.kI = 0.000001; //
          intakePivotLeftConfig.Slot0.kV = 0.12; //
          intakePivotLeftConfig.Slot0.kD = 0.02; //
          intakePivotLeftConfig.CurrentLimits.SupplyCurrentLimit = 15/3;//climbed on 5
          intakePivotLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          intakePivotLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.MotionMagic.MotionMagicCruiseVelocity = 80 ; //
          intakePivotLeftConfig.MotionMagic.MotionMagicAcceleration = 100; //
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 50;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
          intakePivotLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          intakePivotLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          return intakePivotLeftConfig;
        }

        public static TalonFXConfiguration getArmPivotConfiguration(){
          TalonFXConfiguration intakePivotLeftConfig = new TalonFXConfiguration();
          intakePivotLeftConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          intakePivotLeftConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          intakePivotLeftConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          intakePivotLeftConfig.Slot0.kP = 5;// was 8
          intakePivotLeftConfig.Slot0.kI = 0.000001; //
          intakePivotLeftConfig.Slot0.kV = 0.12; //
          intakePivotLeftConfig.Slot0.kD = 0.02; //
          intakePivotLeftConfig.CurrentLimits.SupplyCurrentLimit = 20;//climbed on 5
          intakePivotLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          intakePivotLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.MotionMagic.MotionMagicCruiseVelocity = 80 ; //
          intakePivotLeftConfig.MotionMagic.MotionMagicAcceleration = 80; // 10-15 previously 60
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 26.0;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
          intakePivotLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          intakePivotLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          return intakePivotLeftConfig;
        }

        
        public static TalonFXConfiguration getArmPivotConfigurationFast(){
          TalonFXConfiguration intakePivotLeftConfig = new TalonFXConfiguration();
          intakePivotLeftConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          intakePivotLeftConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          intakePivotLeftConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          intakePivotLeftConfig.Slot0.kP = 5;// was 8
          intakePivotLeftConfig.Slot0.kI = 0.000001; //
          intakePivotLeftConfig.Slot0.kV = 0.12; //
          intakePivotLeftConfig.Slot0.kD = 0.02; //
          intakePivotLeftConfig.CurrentLimits.SupplyCurrentLimit = 20;//climbed on 5
          intakePivotLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          intakePivotLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.MotionMagic.MotionMagicCruiseVelocity = 90 ; //
          intakePivotLeftConfig.MotionMagic.MotionMagicAcceleration = 150; //
          //intakePivotLeftConfig.Slot0.MotionMagic.MotionMagicAcceleration = 10; //
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 26.0;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
          intakePivotLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          intakePivotLeftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
          return intakePivotLeftConfig;
        }


  }

}
