// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import com.ctre.phoenix6.*;
//import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
//import com.ctre.phoenix6.configs.MotorOutputConfigs;
//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
/*import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.hardware.TalonFX;
*/
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

  public static final double scoringPositionTolerance = 0.1; // Scoring position tolerance for superstructure
  public static final double positionTolerance = 0.2; // general position tolerance for superstructure
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
  public static final int intakePivotLeftID = 18;
  public static final int intakePivotRightID = 21;
  public static final int intakeWheelsID = 19;
  public static final int funnelWheelsID = 20;

/* Clearance positions*/
  public static final double intakePivotGearRatio = 270; // 200:1 gear ratio
  public static final double endEffectorPivotGearRatio = 60 / 8 * 4; // 30:1 gear ratio
  public static final double crossbarClearancePos = 4; //elevator height to clear crossbar (must be below)
  public static final double intakeEndeffectorClearancePos = 3; //intake pivot position to clear the endeffector (intake must be deployed enough)
  public static final double endeffectorElevatorClearancePos = 10 * endEffectorPivotGearRatio / 360; //Endeffector pivot position to clear elevator
  
  

  public static class ReefScoringLocations{
      // REEF PRE-SCORING POSES
      public static   final Pose2d  BLUEPRESCORE_1  = new Pose2d( 6.417, 3.826,   Rotation2d.fromDegrees(   	180  	)); 
      public static   final Pose2d  BLUEPRESCORE_2  = new Pose2d( 6.418, 4.217,   Rotation2d.fromDegrees(   	180 	)); 
      public static   final Pose2d  BLUEPRESCORE_3  = new Pose2d( 5.629, 5.585,   Rotation2d.fromDegrees(    -120          ));   
      public static   final Pose2d  BLUEPRESCORE_4  = new Pose2d( 5.289, 5.801,   Rotation2d.fromDegrees(    -120          ));   
      public static   final Pose2d  BLUEPRESCORE_5  = new Pose2d( 3.703, 5.782,   Rotation2d.fromDegrees(    	-60	         ));  
      public static   final Pose2d  BLUEPRESCORE_6  = new Pose2d( 3.360, 5.612,   Rotation2d.fromDegrees(    	-60	         ));  
      public static   final Pose2d  BLUEPRESCORE_7  = new Pose2d( 2.561, 4.217,   Rotation2d.fromDegrees(   	0	   )); 
      public static   final Pose2d  BLUEPRESCORE_8  = new Pose2d( 2.562, 3.826,   Rotation2d.fromDegrees(   	0	   )); 
      public static   final Pose2d  BLUEPRESCORE_9  = new Pose2d( 3.351, 2.451,   Rotation2d.fromDegrees(   	60 	)); 
      public static   final Pose2d  BLUEPRESCORE_10 = new Pose2d( 3.689, 2.262,   Rotation2d.fromDegrees(   	60 	)); 
      public static   final Pose2d  BLUEPRESCORE_11 = new Pose2d( 5.283, 2.255,   Rotation2d.fromDegrees(   	120 	)); 
      public static   final Pose2d  BLUEPRESCORE_12 = new Pose2d( 5.620, 2.450,   Rotation2d.fromDegrees(   	120 	)); 

      public	static	final	Pose2d	REDPRESCORE_1	  =	new	Pose2d(	11.118,	4.220,  Rotation2d.fromDegrees( 0    )); 
      public	static	final	Pose2d	REDPRESCORE_2	  =	new	Pose2d(	11.117,	3.829,  Rotation2d.fromDegrees( 0    )); 
      public	static	final	Pose2d	REDPRESCORE_3	  =	new	Pose2d(	11.906,	2.461,  Rotation2d.fromDegrees( 60   )); 
      public	static	final	Pose2d	REDPRESCORE_4	  =	new	Pose2d(	12.246,	2.245,  Rotation2d.fromDegrees( 60   )); 
      public	static	final	Pose2d	REDPRESCORE_5	  =	new	Pose2d(	13.832,	2.264,  Rotation2d.fromDegrees( 120  )); 
      public	static	final	Pose2d	REDPRESCORE_6	  =	new	Pose2d(	14.175,	2.434,  Rotation2d.fromDegrees( 120  )); 
      public	static	final	Pose2d	REDPRESCORE_7	  =	new	Pose2d(	14.974,	3.829,  Rotation2d.fromDegrees( 180  ));     
      public	static	final	Pose2d	REDPRESCORE_8	  =	new	Pose2d(	14.973,	4.220,  Rotation2d.fromDegrees( 180  ));     
      public	static	final	Pose2d	REDPRESCORE_9	  =	new	Pose2d(	14.184,	5.595,  Rotation2d.fromDegrees( -120 ));	            
      public	static	final	Pose2d	REDPRESCORE_10	=	new	Pose2d(	13.846,	5.784,  Rotation2d.fromDegrees( -120 ));	            
      public	static	final	Pose2d	REDPRESCORE_11	=	new	Pose2d(	12.252,	5.791,  Rotation2d.fromDegrees( -60  ));	             
      public	static	final	Pose2d	REDPRESCORE_12	=	new	Pose2d(	11.915,	5.596,  Rotation2d.fromDegrees( -60  ));	             

  // REEF SCORING POSES
      public static   final Pose2d  BLUE_1          = new Pose2d( 6.119, 3.857,   Rotation2d.fromDegrees( 	180 	)); 
      public static   final Pose2d  BLUE_2          = new Pose2d( 6.119, 4.187,   Rotation2d.fromDegrees( 	180 	)); 
      public static   final Pose2d  BLUE_3          = new Pose2d( 5.452, 5.343,   Rotation2d.fromDegrees(  -120          ));  
      public static   final Pose2d  BLUE_4          = new Pose2d( 5.166, 5.527,   Rotation2d.fromDegrees(  -120          ));  
      public static   final Pose2d  BLUE_5          = new Pose2d( 3.826, 5.508,   Rotation2d.fromDegrees(  	-60	         ));  
      public static   final Pose2d  BLUE_6          = new Pose2d( 3.534, 5.368,   Rotation2d.fromDegrees(  	-60	         ));  
      public static   final Pose2d  BLUE_7          = new Pose2d( 2.860, 4.187,   Rotation2d.fromDegrees( 	0	   )); 
      public static   final Pose2d  BLUE_8          = new Pose2d( 2.860, 3.857,   Rotation2d.fromDegrees( 	0 	  )); 
      public static   final Pose2d  BLUE_9          = new Pose2d( 3.527, 2.694,   Rotation2d.fromDegrees( 	60 	)); 
      public static   final Pose2d  BLUE_10         = new Pose2d( 3.813, 2.535,   Rotation2d.fromDegrees( 	60 	)); 
      public static   final Pose2d  BLUE_11         = new Pose2d( 5.160, 2.529,   Rotation2d.fromDegrees( 	120 	)); 
      public static   final Pose2d  BLUE_12         = new Pose2d( 5.445, 2.694,   Rotation2d.fromDegrees( 	120 	)); 

      public	static	final	Pose2d	RED_1	          =	new	Pose2d(	11.416,	4.189,	Rotation2d.fromDegrees(	 0    )); 
      public	static	final	Pose2d	RED_2	          =	new	Pose2d(	11.416,	3.859,	Rotation2d.fromDegrees(	 0    )); 
      public	static	final	Pose2d	RED_3	          =	new	Pose2d(	12.083,	2.703,	Rotation2d.fromDegrees(	 60   )); 
      public	static	final	Pose2d	RED_4	          =	new	Pose2d(	12.369,	2.519,	Rotation2d.fromDegrees(	 60   )); 
      public	static	final	Pose2d	RED_5	          =	new	Pose2d(	13.709,	2.538,	Rotation2d.fromDegrees(	 120  )); 
      public	static	final	Pose2d	RED_6	          =	new	Pose2d(	14.001,	2.678,	Rotation2d.fromDegrees(	 120  )); 
      public	static	final	Pose2d	RED_7	          =	new	Pose2d(	14.8,	4.03,	Rotation2d.fromDegrees(	 180  )); 
      public	static	final	Pose2d	RED_8	          =	new	Pose2d(	14.8,	4.3,	Rotation2d.fromDegrees(	 180  )); 
      public	static	final	Pose2d	RED_9	          =	new	Pose2d(	14.008,	5.352,	Rotation2d.fromDegrees(  -120 ));	        
      public	static	final	Pose2d	RED_10	        =	new	Pose2d(	13.722,	5.511,	Rotation2d.fromDegrees(  -120 ));	        
      public	static	final	Pose2d	RED_11	        =	new	Pose2d(	12.375,	5.517,	Rotation2d.fromDegrees(	 -60  ));	        
      public	static	final	Pose2d	RED_12	        =	new	Pose2d(	12.090,	5.352,	Rotation2d.fromDegrees(	 -60  ));	        
      
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
    }
    return prescoreDrivePose;
      }
  }

  public static class SuperstructureConfigs{
    
        public static TalonFXConfiguration getElevatorConfigLeft() {
          TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration();
          elevatorConfigLeft.Slot0.kG = 0.35; // Volts to overcome gravity
          elevatorConfigLeft.Slot0.kS = 0.0; // Volts to overcome static friction
          elevatorConfigLeft.Slot0.kV = 0.12; // Volts for a velocity target of 1 rps
          elevatorConfigLeft.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          elevatorConfigLeft.Slot0.kP = 10;//30 was too high
          elevatorConfigLeft.Slot0.kI = 0.00001; //
          elevatorConfigLeft.Slot0.kD = 0.01; //
          
          elevatorConfigLeft.CurrentLimits.SupplyCurrentLimit = 25;//was 20
          elevatorConfigLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;
          elevatorConfigLeft.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          elevatorConfigLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          elevatorConfigLeft.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          elevatorConfigLeft.MotionMagic.MotionMagicCruiseVelocity = 80; //100 was smooth, 200 is faster than kraken max 100
          elevatorConfigLeft.MotionMagic.MotionMagicAcceleration = 250;// 80 was good, 200 zippy
          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          elevatorConfigLeft.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 76;
          elevatorConfigLeft.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5;
          elevatorConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return elevatorConfigLeft;
        }
        public static TalonFXConfiguration getElevatorConfigRight() {
          TalonFXConfiguration elevatorConfigRight = new TalonFXConfiguration();
          elevatorConfigRight = getElevatorConfigLeft();
          //elevatorConfigRight.Slot0.kG = 0.3; // Volts to overcome gravity
          //elevatorConfigRight.Slot0.kS = 0.4; // Volts to overcome static friction
          //elevatorConfigRight.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
          //elevatorConfigRight.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          //elevatorConfigRight.Slot0.kP = 10;//
          //elevatorConfigRight.Slot0.kI = 0.01; //
          //elevatorConfigRight.Slot0.kV = 0.12; //
          //elevatorConfigRight.Slot0.kD = 0.005; //
          //elevatorConfigRight.CurrentLimits.SupplyCurrentLimit = 30 / 3;//was 20
          //elevatorConfigRight.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
          //elevatorConfigRight.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
          //elevatorConfigRight.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
          //elevatorConfigRight.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
          //elevatorConfigRight.MotionMagic.MotionMagicCruiseVelocity = 350 / 1; //stolen from 3255, added '/10' to start slow
          //elevatorConfigRight.MotionMagic.MotionMagicAcceleration = 2500 / 1.5; //also stolen from 3255, added '/10' to start slow
          elevatorConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          return elevatorConfigRight;
        }

        public static TalonFXConfiguration getEndeffectorPivotConfig(){
          TalonFXConfiguration endeffectorPivotConfig = new TalonFXConfiguration();
          endeffectorPivotConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorPivotConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorPivotConfig.Slot0.kV = 0.12; //
          endeffectorPivotConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s

          endeffectorPivotConfig.Slot0.kP = 20;//
          endeffectorPivotConfig.Slot0.kI = 0.00001; //
          endeffectorPivotConfig.Slot0.kD = 0.05; //

          endeffectorPivotConfig.CurrentLimits.SupplyCurrentLimit = 8;//
          endeffectorPivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          endeffectorPivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          endeffectorPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          endeffectorPivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          endeffectorPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 80; //20 was smooth
          endeffectorPivotConfig.MotionMagic.MotionMagicAcceleration = 150; //50 was smooth
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
          intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 30;//
          intakeWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
          intakeWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
          intakeWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
          intakeWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
          intakeWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 35; //stolen from 3255
          intakeWheelsConfig.MotionMagic.MotionMagicAcceleration = 250; //also stolen from 3255
          return intakeWheelsConfig;
        }

        public static TalonFXConfiguration getEndeffectorWheelsConfiguration(){
          TalonFXConfiguration endeffectorWheelsConfig = new TalonFXConfiguration();
          endeffectorWheelsConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          endeffectorWheelsConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          endeffectorWheelsConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          endeffectorWheelsConfig.Slot0.kP = 5;//
          endeffectorWheelsConfig.Slot0.kI = 0.01; //
          endeffectorWheelsConfig.Slot0.kV = 0.12; //
          endeffectorWheelsConfig.Slot0.kD = 0.01; //
          endeffectorWheelsConfig.CurrentLimits.SupplyCurrentLimit = 20;//
          endeffectorWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .2;        
          endeffectorWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;
          endeffectorWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .2;
          endeffectorWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .2;
          endeffectorWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 50; //stolen from 3255
          endeffectorWheelsConfig.MotionMagic.MotionMagicAcceleration = 50; //also stolen from 3255
          return endeffectorWheelsConfig;
        }

        public static TalonFXConfiguration getFunnelWheelsConfiguration(){
          TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
          funnelConfig.Slot0.kG = 0.3; // Volts to overcome gravity
          funnelConfig.Slot0.kS = 0.4; // Volts to overcome static friction
          funnelConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
          funnelConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          funnelConfig.Slot0.kP = 20;//
          funnelConfig.Slot0.kI = 0.01; //
          funnelConfig.Slot0.kV = 0.12; //
          funnelConfig.Slot0.kD = 0.00001; //
          funnelConfig.CurrentLimits.SupplyCurrentLimit = 15;//was 20
          funnelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
          funnelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
          funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
          funnelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
          funnelConfig.MotionMagic.MotionMagicCruiseVelocity = 350 / 10; //stolen from 3255, added '/10' to start slow
          funnelConfig.MotionMagic.MotionMagicAcceleration = 2500 / 10; //also stolen from 3255, added '/10' to start slow
          return funnelConfig;
        }

        public static TalonFXConfiguration getIntakePivotLeftConfiguration(){
          TalonFXConfiguration intakePivotLeftConfig = new TalonFXConfiguration();
          intakePivotLeftConfig.Slot0.kG = 0.0; // Volts to overcome gravity
          intakePivotLeftConfig.Slot0.kS = 0.0; // Volts to overcome static friction
          intakePivotLeftConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
          intakePivotLeftConfig.Slot0.kP = 8;//
          intakePivotLeftConfig.Slot0.kI = 0.0001; //
          intakePivotLeftConfig.Slot0.kV = 0.12; //
          intakePivotLeftConfig.Slot0.kD = 0.02; //
          intakePivotLeftConfig.CurrentLimits.SupplyCurrentLimit = 5;//
          intakePivotLeftConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .02;        
          intakePivotLeftConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .02;
          intakePivotLeftConfig.MotionMagic.MotionMagicCruiseVelocity = 200; //
          intakePivotLeftConfig.MotionMagic.MotionMagicAcceleration = 250; //
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
          intakePivotLeftConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 92.3;
          intakePivotLeftConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
          intakePivotLeftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
          return intakePivotLeftConfig;
        }

        public static TalonFXConfiguration getIntakePivotRightConfiguration(){
          TalonFXConfiguration intakePivotRightConfig = new TalonFXConfiguration();
          intakePivotRightConfig = getIntakePivotLeftConfiguration();
          //intakePivotRightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
          return intakePivotRightConfig;
        }


  }
}
