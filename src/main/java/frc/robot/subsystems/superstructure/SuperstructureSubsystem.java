// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
  private static final TalonFX mFunnelWheels = new TalonFX(Constants.funnelWheelsID);//

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_ElevatorLeft;
  private final MechanismLigament2d m_ElevatorRight;
  private final MechanismLigament2d m_EndeffectorPivot;
  private final MechanismLigament2d m_EndeffectorRollers;
  private final MechanismLigament2d m_IntakeLeftPivot;
  private final MechanismLigament2d m_IntakeRightPivot;
  private final MechanismLigament2d m_IntakeWheels;
  private final MechanismLigament2d m_FunnelWheels;

  /** Creates a new ExampleSubsystem. */
  public SuperstructureSubsystem() {

    /* Configurations */
        mElevatorRight.getConfigurator().apply(new TalonFXConfiguration());
        mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeft());
        mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig());
        mEndeffectorRollers.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakeWheelsConfiguration());//using intake config for now, should be similar. Once we tune one of the EE rollers / funnel / intake rollers, we can copy that starting point for the others
        mIntakePivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig()); //using EE pivot for now
        mIntakeWheels.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakeWheelsConfiguration());
        mFunnelWheels.getConfigurator().apply(Constants.SuperstructureConfigs.getFunnelWheelsConfiguration());
        mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true)); //BVN 1-31-25, of we have any issue with the left leader right follower, they can just both go to the same pos

        /*TalonFXConfiguration elevatorConfigLeft = new TalonFXConfiguration(); //all of the commented sections are added to constants now
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
        
        //TalonFXConfiguration elevatorConfigRight = Constants.SuperstructureConfigs.getElevatorConfigLeft();
        //elevatorConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

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
        
        TalonFXConfiguration intakeWheelsConfig = new TalonFXConfiguration();
        intakeWheelsConfig.Slot0.kG = 0.3; // Volts to overcome gravity
        intakeWheelsConfig.Slot0.kS = 0.4; // Volts to overcome static friction
        intakeWheelsConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        intakeWheelsConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        intakeWheelsConfig.Slot0.kP = 0.1;//
        intakeWheelsConfig.Slot0.kI = 0.01; //
        intakeWheelsConfig.Slot0.kV = 0.12; //
        intakeWheelsConfig.Slot0.kD = 0.00001; //
        intakeWheelsConfig.CurrentLimits.SupplyCurrentLimit = 30.0 / 10;//
        intakeWheelsConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        intakeWheelsConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        intakeWheelsConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        intakeWheelsConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        intakeWheelsConfig.MotionMagic.MotionMagicCruiseVelocity = 350 / 10; //stolen from 3255
        intakeWheelsConfig.MotionMagic.MotionMagicAcceleration = 2500 / 10; //also stolen from 3255

        mIntakeWheels.getConfigurator().apply(intakeWheelsConfig);

        TalonFXConfiguration funnelConfig = new TalonFXConfiguration();
        funnelConfig.Slot0.kG = 0.3; // Volts to overcome gravity
        funnelConfig.Slot0.kS = 0.4; // Volts to overcome static friction
        funnelConfig.Slot0.kV = 0.001; // Volts for a velocity target of 1 rps
        funnelConfig.Slot0.kA = 0.001; // Volts for an acceleration of 1 rps/s
        funnelConfig.Slot0.kP = 0.1;//
        funnelConfig.Slot0.kI = 0.01; //
        funnelConfig.Slot0.kV = 0.12; //
        funnelConfig.Slot0.kD = 0.00001; //
        funnelConfig.CurrentLimits.SupplyCurrentLimit = 30 / 10;//was 20
        funnelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = .08;        
        funnelConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .08;
        funnelConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = .08;
        funnelConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = .08;
        funnelConfig.MotionMagic.MotionMagicCruiseVelocity = 350 / 10; //stolen from 3255, added '/10' to start slow
        funnelConfig.MotionMagic.MotionMagicAcceleration = 2500 / 10; //also stolen from 3255, added '/10' to start slow

        mFunnelWheels.getConfigurator().apply(funnelConfig);*/

         /* Mechanism2d */
        mech = new Mechanism2d(27, 100);
        root = mech.getRoot("Center", 13.5, 13.5);
        MechanismRoot2d elevatorLeftRoot = mech.getRoot("ElevatorLeftRoot", 1, 9.375);
        MechanismRoot2d elevatorRightRoot = mech.getRoot("ElevatorRightRoot", 26, 9.375);
        MechanismRoot2d endEffectorPivot = mech.getRoot("EndEffectorPivotRoot", 1.0, 9.375);
        MechanismRoot2d intakePivotLeftRoot = mech.getRoot("IntakePivotLeftRoot", 1.0, 9.375);
        MechanismRoot2d intakePivotRightRoot = mech.getRoot("IntakePivotRightRoot", 1.0, 9.375);
        m_ElevatorLeft = elevatorLeftRoot.append(new MechanismLigament2d("ElevatorLeft", 96.75, 90));
        m_ElevatorRight = elevatorRightRoot.append(new MechanismLigament2d("ElevatorRight", 96.75, 90));
        m_EndeffectorPivot = endEffectorPivot.append(new MechanismLigament2d("EndeffectorPivot", 96.75, 90));
        m_EndeffectorRollers = endEffectorPivot.append(new MechanismLigament2d("EndeffectorRollers", 96.75, 90));
        m_IntakeLeftPivot = intakePivotLeftRoot.append(new MechanismLigament2d("IntakePivot", 96.75, 45));
        m_IntakeRightPivot = intakePivotRightRoot.append(new MechanismLigament2d("IntakePivot", 96.75, 45));
        m_IntakeWheels = intakePivotLeftRoot.append(new MechanismLigament2d("IntakeWheels", 25, 90));
        m_FunnelWheels = root.append(new MechanismLigament2d("FunnelWheels", 96.75, 90));
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

  public void setPreScoreCoralPos(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}

  public void setScoreCoralPos(Double level){ //sets up elevator and EE pivot for score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;
    if (level == 1) TARGETSTATE = STATE.CoralL1;
    if (level == 2) TARGETSTATE = STATE.CoralL2;
    if (level == 3) TARGETSTATE = STATE.CoralL3;
    if (level == 4) TARGETSTATE = STATE.CoralL4;
  }

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

    MECH2d();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void MECH2d(){
    SmartDashboard.putData("MyMechanism", mech);
    SmartDashboard.putNumber("ElevatorLeft", mElevatorLeft.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ElevatorRight", mElevatorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("EndeffectorPivot", mEndeffectorPivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("IntakePivot", mIntakePivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("EndeffectorRollers", mEndeffectorRollers.getPosition().getValueAsDouble()); 
    SmartDashboard.putNumber("IntakeWheels", mIntakeWheels.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("FunnelWheels", mFunnelWheels.getPosition().getValueAsDouble());

    m_ElevatorLeft.setLength(mElevatorLeft.getPosition().getValueAsDouble());
    m_ElevatorRight.setLength(mElevatorRight.getPosition().getValueAsDouble());
    m_EndeffectorPivot.setAngle(mEndeffectorPivot.getPosition().getValueAsDouble());
    m_IntakeRightPivot.setAngle(mIntakePivot.getPosition().getValueAsDouble());
    m_IntakeLeftPivot.setAngle(mIntakePivot.getPosition().getValueAsDouble());
    m_EndeffectorRollers.setLength(mEndeffectorRollers.getPosition().getValueAsDouble());
    m_IntakeWheels.setLength(mIntakeWheels.getPosition().getValueAsDouble());
    m_FunnelWheels.setLength(mFunnelWheels.getPosition().getValueAsDouble());
  }
}
