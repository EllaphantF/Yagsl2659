// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class EndeffectorElevatorSubsystem extends SubsystemBase {

    public double elevatorTarget = 0;
    public double pivotTarget = 0;
    public boolean hasCoral = false;
    public boolean hasAlgae = false;

    private static final TalonFX mElevatorRight = new TalonFX(1);//BVN 1-26-25 need to put IDs in constants and link them here
    private static final TalonFX mElevatorLeft = new TalonFX(1);//
    private static final TalonFX mEndeffectorPivot = new TalonFX(1);//
    private static final TalonFX mEndeffectorRollers = new TalonFX(1);// */

  /** Creates a new ExampleSubsystem. */
  /* public EndeffectorElevatorSubsystem() {
        mElevatorRight.getConfigurator().apply(new TalonFXConfiguration());
        mElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());
        mEndeffectorPivot.getConfigurator().apply(new TalonFXConfiguration());
        mEndeffectorRollers.getConfigurator().apply(new TalonFXConfiguration());
        
        TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration();
        elevatorConfigLeft.Slot0.kG = 0.3; // Volts to overcome gravity
        elevatorConfigLeft.Slot0.kS = 0.4; // Volts to overcome static friction
        elevatorConfigLeft.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        elevatorConfigLeft.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        elevatorConfigLeft.Slot0.kP = 0.1;//
        elevatorConfigLeft.Slot0.kI = 0.01; //
        elevatorConfigLeft.Slot0.kV = 0.12; //
        elevatorConfigLeft.Slot0.kD = 0.00001; //
        elevatorConfigLeft.CurrentLimits.SupplyCurrentLimit = 30.0;//was 20
        elevatorConfigLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        elevatorConfigLeft.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        elevatorConfigLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        elevatorConfigLeft.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        elevatorConfigLeft.MotionMagic.MotionMagicCruiseVelocity = 350; //stolen from 3255
        elevatorConfigLeft.MotionMagic.MotionMagicAcceleration = 2500; //also stolen from 3255

        elevatorConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfiguration elevatorConfigLeftRight = elevatorConfigLeft;
        elevatorConfigLeftRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        mElevatorLeft.getConfigurator().apply(elevatorConfigLeft);
        mElevatorRight.getConfigurator().apply(elevatorConfigLeftRight);
        
        TalonFXConfiguration endeffectorPivotConfig = new TalonFXConfiguration();
        endeffectorPivotConfig.Slot0.kG = 0.3; // Volts to overcome gravity
        endeffectorPivotConfig.Slot0.kS = 0.4; // Volts to overcome static friction
        endeffectorPivotConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        endeffectorPivotConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        endeffectorPivotConfig.Slot0.kP = 0.1;//
        endeffectorPivotConfig.Slot0.kI = 0.01; //
        endeffectorPivotConfig.Slot0.kV = 0.12; //
        endeffectorPivotConfig.Slot0.kD = 0.00001; //
        endeffectorPivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0;//was 20
        endeffectorPivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        endeffectorPivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        endeffectorPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        endeffectorPivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        endeffectorPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 350; //stolen from 3255
        endeffectorPivotConfig.MotionMagic.MotionMagicAcceleration = 2500; //also stolen from 3255

        mEndeffectorPivot.getConfigurator().apply(endeffectorPivotConfig);
  

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  //public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
  //  return runOnce(
  //      () -> {
          /* one-time action goes here */
  //      });
  //} */

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  /* public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  public void motionMagicSetElevatorAndEndeffector(double ElevatorPosTarget, double PivotPosTarget){
    mElevatorLeft.setControl(new MotionMagicVoltage(ElevatorPosTarget));
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
    mEndeffectorPivot.setControl(new MotionMagicVoltage(PivotPosTarget));
  }

  public void manualUpdateTargets(double ElevatorPosTarget, double PivotPosTarget){
   elevatorTarget = ElevatorPosTarget;
   pivotTarget = PivotPosTarget;
  }

  public void stow(){ //pulls the intake in and elevator down
    if(hasCoral){
      elevatorTarget = 0; //reference constants or arm pos file or replace these with states
      pivotTarget = 0;
      //STATE = STOW_W_CORAL; //option for putting positions in a state
    }
    else if (hasAlgae) {
      elevatorTarget = 0; //reference constants or arm pos file 
      pivotTarget = 0;
    }
    else{
      elevatorTarget = 0; //reference constants or arm pos file 
      pivotTarget = 0;
    }
  }

  public void setPreScoreCoralPos(Integer level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    elevatorTarget = 0; //reference constants or arm pos file 
    pivotTarget = 0;
  }

  public void setScoreCoralPos(Integer level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    elevatorTarget = 0; //reference constants or arm pos file 
    pivotTarget = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motionMagicSetElevatorAndEndeffector(elevatorTarget, pivotTarget);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
} */
