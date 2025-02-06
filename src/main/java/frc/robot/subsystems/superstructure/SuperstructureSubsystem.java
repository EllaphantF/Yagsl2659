// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class SuperstructureSubsystem extends SubsystemBase {

  public double elevatorTarget = 0;
  public double pivotTarget = 0;
  public double intakeTarget = 0;
  public final double intake = -20;
  public double sequenceState = 0;
  public double scoreLevel = 0;
  public boolean hasCoral = false;
  public boolean hasAlgae = false;
  
  public boolean intakeTraversing = false;
  public boolean homing = false;
  public boolean lifting = false;
  public boolean scoringCoral = false;

  public SuperstructureStates STATE = new SuperstructureStates();
  public SuperstructureState TARGETSTATE = STATE.StartingConfig;

  private static final TalonFX mElevatorRight = new TalonFX(Constants.elevatorRightID);//MPF Hi I have IDs
  private static final TalonFX mElevatorLeft = new TalonFX(Constants.elevatorLeftID);//
  private static final TalonFX mEndeffectorPivot = new TalonFX(Constants.endEffectorPivotID);//
  private static final TalonFX mEndeffectorRollers = new TalonFX(Constants.endEffectorWheelID);//
  private static final TalonFX mIntakePivot = new TalonFX(Constants.intakePivotID);//
  private static final TalonFX mIntakeWheels = new TalonFX(Constants.intakeWheelsID);//

  /** Creates a new ExampleSubsystem. */
  public SuperstructureSubsystem() {
        mElevatorRight.getConfigurator().apply(new TalonFXConfiguration());
        mElevatorLeft.getConfigurator().apply(new TalonFXConfiguration());
        mEndeffectorPivot.getConfigurator().apply(new TalonFXConfiguration());
        mEndeffectorRollers.getConfigurator().apply(new TalonFXConfiguration());
        mIntakePivot.getConfigurator().apply(new TalonFXConfiguration());
        mIntakeWheels.getConfigurator().apply(new TalonFXConfiguration());
        
        TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration();
        elevatorConfigLeft.Slot0.kG = 0.3; // Volts to overcome gravity
        elevatorConfigLeft.Slot0.kS = 0.4; // Volts to overcome static friction
        elevatorConfigLeft.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        elevatorConfigLeft.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        elevatorConfigLeft.Slot0.kP = 0.1;//
        elevatorConfigLeft.Slot0.kI = 0.01; //
        elevatorConfigLeft.Slot0.kV = 0.12; //
        elevatorConfigLeft.Slot0.kD = 0.00001; //
        elevatorConfigLeft.CurrentLimits.SupplyCurrentLimit = 30 / 10;//was 20
        elevatorConfigLeft.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        elevatorConfigLeft.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        elevatorConfigLeft.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        elevatorConfigLeft.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        elevatorConfigLeft.MotionMagic.MotionMagicCruiseVelocity = 350 / 10; //stolen from 3255, added '/10' to start slow
        elevatorConfigLeft.MotionMagic.MotionMagicAcceleration = 2500 / 10; //also stolen from 3255, added '/10' to start slow

        elevatorConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfiguration elevatorConfigRight = elevatorConfigLeft;
        elevatorConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        mElevatorLeft.getConfigurator().apply(elevatorConfigLeft);
        mElevatorRight.getConfigurator().apply(elevatorConfigRight);
        mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true)); //BVN 1-31-25, of we have any issue with the left leader right follower, they can just both go to the same pos
    
        
        TalonFXConfiguration endeffectorPivotConfig = new TalonFXConfiguration();
        endeffectorPivotConfig.Slot0.kG = 0.3; // Volts to overcome gravity
        endeffectorPivotConfig.Slot0.kS = 0.4; // Volts to overcome static friction
        endeffectorPivotConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        endeffectorPivotConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        endeffectorPivotConfig.Slot0.kP = 0.1;//
        endeffectorPivotConfig.Slot0.kI = 0.01; //
        endeffectorPivotConfig.Slot0.kV = 0.12; //
        endeffectorPivotConfig.Slot0.kD = 0.00001; //
        endeffectorPivotConfig.CurrentLimits.SupplyCurrentLimit = 30.0 / 10;//
        endeffectorPivotConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        endeffectorPivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        endeffectorPivotConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        endeffectorPivotConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        endeffectorPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 350 / 10; //stolen from 3255
        endeffectorPivotConfig.MotionMagic.MotionMagicAcceleration = 2500 / 10; //also stolen from 3255

        mEndeffectorPivot.getConfigurator().apply(endeffectorPivotConfig);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public void motionMagicSetElevatorAndEndeffector(double ElevatorPosTarget, double PivotPosTarget, double IntakePosTarget){
    mElevatorLeft.setControl(new MotionMagicVoltage(ElevatorPosTarget));
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
    mEndeffectorPivot.setControl(new MotionMagicVoltage(PivotPosTarget));
    mIntakePivot.setControl(new MotionMagicVoltage(IntakePosTarget));
  }

  public void manualUpdateTargets(double ElevatorPosTarget, double PivotPosTarget, double IntakePosTarget){
   elevatorTarget = ElevatorPosTarget;
   pivotTarget = PivotPosTarget;
   intakeTarget = IntakePosTarget;
  }

  public void intake(){
    intakeTraversing = true;
    setEndeffectorWheelSpeed(intake);
    setIntakeWheelSpeed(intake);
    /* This block checks that the system is in a safe spot before it commands motion, and if there's potential interference(i.e. endeffector would crash) then it goes to a safe stow position first until its below a safety threshold */
    if(mElevatorLeft.getPosition().getValueAsDouble() > Constants.crossbarClearancePos){
      if(mIntakePivot.getPosition().getValueAsDouble() < Constants.intakeEndeffectorClearancePos){
        TARGETSTATE = STATE.StowClear;}
      else{
        TARGETSTATE = STATE.StowClearIntakeDeployed;
      }
    }
    else {
      TARGETSTATE = STATE.Intake;
      intakeTraversing = false;}
  }

  public void setEndeffectorWheelSpeed(double wheelSpeed){
    mEndeffectorRollers.setControl(new VelocityVoltage(wheelSpeed));
  }

  public void setIntakeWheelSpeed(double wheelSpeed){
    mIntakeWheels.setControl(new VelocityVoltage(wheelSpeed));
  }

  public Boolean safeToIntake(){
    Boolean safe = false;
    return safe;
  }

  public Boolean safeToStow(){
    Boolean safe = true;
    if(mElevatorLeft.getPosition().getValueAsDouble() > Constants.crossbarClearancePos && mIntakePivot.getPosition().getValueAsDouble() < Constants.endeffectorElevatorClearancePos){
      safe = false;
    }
    return safe;
  }

  public Boolean safeToLift(){
    Boolean safe = false;
    if(mEndeffectorPivot.getPosition().getValueAsDouble() > Constants.endeffectorElevatorClearancePos){
      safe = true;
    }
    return safe;
  }

  public void clearMotionStates(){
    homing = false;
    intakeTraversing = false;
    lifting = false;
  }

  public void goHome(){
    homing = true;
    if(safeToStow()){
      TARGETSTATE = STATE.Home;
    }
  }

  public void stow(){ //pulls the intake in and elevator down
    if(hasCoral){
      // elevatorTarget = 0; //reference constants or arm pos file or replace these with states
      // pivotTarget = 0;
      TARGETSTATE = STATE.StowCoral;
      // STATE.StowCoral; //option for putting positions in a state
    }
    else if (hasAlgae) {
      // elevatorTarget = 0; //reference constants or arm pos file 
      // pivotTarget = 0;
      TARGETSTATE = STATE.StowAlgae;
    }
    else{
      // elevatorTarget = 0; //reference constants or arm pos file 
      // pivotTarget = 0;
      goHome();
    }
  }
/**
 * Sets superstructure to prescore state. Subsystem handles the motion 
 * 
 * @param level //1 through 4 
 */
  public void setPreScoreCoralPos(Double level){ //
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}

    /**
     * Sets superstructure to scoring state. Subsystem handles the motion
     * @param level
     */
  public void setScoreCoralPos(Double level){ //sets up elevator and EE pivot for score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;
    if (level == 1) TARGETSTATE = STATE.CoralL1;
    if (level == 2) TARGETSTATE = STATE.CoralL2;
    if (level == 3) TARGETSTATE = STATE.CoralL3;
    if (level == 4) TARGETSTATE = STATE.CoralL4;
  }
  /**
   * Sets superstructure to post-score state. Subsystem handles the motion 
   * @param level
   */
  public void setPostScoreCoralPos(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}


  public void scoreCoral(Double level){
    scoreLevel = level;
    scoringCoral = true;
    lifting = true;
    sequenceState = 0;
  }

  public void releaseCoral(){
    setEndeffectorWheelSpeed(1);
  }

  public void lift(){
    if(safeToLift()){
      if(scoringCoral){
        if(sequenceState == 0){
          setPreScoreCoralPos(scoreLevel);
          sequenceState = 1;
        }
        else if(sequenceState == 1){
          setScoreCoralPos(scoreLevel);
          //sequenceState = 2;
        }
        else if(sequenceState == 2){
          setScoreCoralPos(scoreLevel);
          sequenceState = 2;
        }

      }
    }
    
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    motionMagicSetElevatorAndEndeffector(TARGETSTATE.elevator, TARGETSTATE.pivot, TARGETSTATE.intake);

    if(intakeTraversing)intake();
    if(homing)stow();
    if(lifting)lift();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}