// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.RobotBase;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.compound.Diff_MotionMagicVoltage_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Position;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class SuperstructureSubsystem extends SubsystemBase {

  private double simulatedMotorMotionTimer = 0;
  public double elevatorTarget = 0;
  public double pivotTarget = 0;
  public double intakeTarget = 0;  
  public double elevatorPos = 0;
  public double pivotPos = 0;
  public double intakePos = 0;
  public double releaseTimer = 0;
  public final double intake = -20;
  public double sequenceState = 0;
  public double scoreLevel = 0;
  public double algaeLevel = 0;
  public boolean hasCoral = false;
  public boolean hasAlgae = false;
  public boolean seatingCoral = false;
  
  
  public static boolean autoReleaseCoral = true;

  public boolean intakeTraversing = false;
  public boolean intaking = false;
  public boolean stowing = false;
  public boolean lifting = false;
  public boolean scoringCoral = false;
  public boolean releasingCoral = false;
  public boolean manualOverride = false;

  public SuperstructureStates STATE = new SuperstructureStates();
  public SuperstructureState TARGETSTATE = STATE.StartingConfig;

  private static final TalonFX mElevatorRight = new TalonFX(Constants.elevatorRightID);//MPF Hi I have IDs
  private static final TalonFX mElevatorLeft = new TalonFX(Constants.elevatorLeftID);//
  private static final TalonFX mEndeffectorPivot = new TalonFX(Constants.endEffectorPivotID);//
  private static final TalonFX mEndeffectorRollers = new TalonFX(Constants.endEffectorWheelID);//
  private static final TalonFX mIntakePivotLeft = new TalonFX(Constants.intakePivotLeftID);//
  private static final TalonFX mIntakePivotRight = new TalonFX(Constants.intakePivotRightID);
  private static final TalonFX mIntakeWheels = new TalonFX(Constants.intakeWheelsID);//
  private static final TalonFX mFunnelWheels = new TalonFX(Constants.funnelWheelsID);//
  private static final CANdi CANdi = new CANdi(25);

  //private final TalonFXSimState mElevatorLeftSim = new TalonFXSimState(mElevatorLeft);
  //private final TalonFXSimState mEndeffectorPivotSim = new TalonFXSimState(mEndeffectorPivot);
  //private final TalonFXSimState mIntakePivotSim = new TalonFXSimState(mIntakePivot);

  private final Mechanism2d mech;
  private final MechanismRoot2d root;
  private final MechanismLigament2d m_ElevatorLeft;
  //private final MechanismLigament2d m_ElevatorRight;
  private final MechanismLigament2d m_EndeffectorPivot;
  private final MechanismLigament2d m_EndeffectorRollers;
  private final MechanismLigament2d m_IntakeLeftPivot;
  //private final MechanismLigament2d m_IntakeRightPivot;
  private final MechanismLigament2d m_IntakeWheels;
  private final MechanismLigament2d m_FunnelWheels;

  /** Creates a new ExampleSubsystem. */
  public SuperstructureSubsystem() {
    /* Configurations */
        mElevatorRight.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigRight());
        mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeft());
        mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig());
        mEndeffectorRollers.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorWheelsConfiguration());//using intake config for now, should be similar. Once we tune one of the EE rollers / funnel / intake rollers, we can copy that starting point for the others
        mIntakePivotLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakePivotLeftConfiguration()); //
        mIntakePivotRight.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakePivotRightConfiguration()); //
        mIntakeWheels.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakeWheelsConfiguration());
        mFunnelWheels.getConfigurator().apply(Constants.SuperstructureConfigs.getFunnelWheelsConfiguration());
        mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true)); //
        mIntakePivotRight.setControl(new Follower(mIntakePivotLeft.getDeviceID(), true));

        

         /* Mechanism2d */
        
        mech = new Mechanism2d(50, 120);
        root = mech.getRoot("Center", 13.5, 13.5);
        MechanismRoot2d elevatorLeftRoot = mech.getRoot("ElevatorLeftRoot", 10, 25);
        //MechanismRoot2d elevatorRightRoot = mech.getRoot("ElevatorRightRoot", 26, 9.375);
        //MechanismRoot2d endEffectorPivot = mech.getRoot("EndEffectorPivotRoot", 10, 40);
        MechanismRoot2d intakePivotLeftRoot = mech.getRoot("IntakePivotLeftRoot", 20, 5);
        //MechanismRoot2d intakePivotRightRoot = mech.getRoot("IntakePivotRightRoot", 1.0, 9.375);
        m_ElevatorLeft = elevatorLeftRoot.append(new MechanismLigament2d("ElevatorLeft", 65, 90));
        //m_ElevatorRight = elevatorRightRoot.append(new MechanismLigament2d("ElevatorRight", 96.75, 90));
        m_EndeffectorPivot = m_ElevatorLeft.append(new MechanismLigament2d("EndeffectorPivot", 12, 270));
        m_EndeffectorRollers = m_ElevatorLeft.append(new MechanismLigament2d("EndeffectorRollers", 3, 90));
        m_IntakeLeftPivot = intakePivotLeftRoot.append(new MechanismLigament2d("IntakePivot", 15, 90));
        //m_IntakeRightPivot = intakePivotRightRoot.append(new MechanismLigament2d("IntakePivot", 96.75, 45));
        m_IntakeWheels = m_IntakeLeftPivot.append(new MechanismLigament2d("IntakeWheels", 3, 90));
        m_FunnelWheels = root.append(new MechanismLigament2d("FunnelWheels", 3, 90));

  }

  public  void climb(double state){
    
    if (state == 1) TARGETSTATE = STATE.climb1;
    if (state == 2) {TARGETSTATE = STATE.climb2; setIntakeWheelSpeed(3);
    }
    if (state == 3) TARGETSTATE = STATE.climb3;
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
    mEndeffectorPivot.setControl(new MotionMagicVoltage(PivotPosTarget ));
    mIntakePivotLeft.setControl(new MotionMagicVoltage(IntakePosTarget ));
    //mIntakePivotRight.setControl(new MotionMagicVoltage(IntakePosTarget * Constants.intakePivotGearRatio / 360));
  }

  public void SD_motionMagicElevatorTEST(){
    SmartDashboard.putNumber("ElevatorTestTarget", SmartDashboard.getNumber("ElevatorTestTarget", mElevatorLeft.getPosition().getValueAsDouble()));
    double ElevatorTestTarget = SmartDashboard.getNumber("ElevatorTestTarget", mElevatorLeft.getPosition().getValueAsDouble());
    mElevatorLeft.setControl(new MotionMagicVoltage(ElevatorTestTarget));
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
  }

  public void updateElevatorConfigsFromSD(){
    // Get the current PID values and motion magic parameters
    TalonFXConfiguration elevatorConfig = Constants.SuperstructureConfigs.getElevatorConfigLeft();
    mElevatorLeft.getConfigurator().refresh(elevatorConfig);

    double kP = elevatorConfig.Slot0.kP;
    double kD = elevatorConfig.Slot0.kD;
    double kI = elevatorConfig.Slot0.kI;
    double kG = elevatorConfig.Slot0.kG;
    double kS = elevatorConfig.Slot0.kS;
    double kV = elevatorConfig.Slot0.kV;
    double kA = elevatorConfig.Slot0.kA;
    double motionMagicVelocity = elevatorConfig.MotionMagic.MotionMagicCruiseVelocity;
    double motionMagicAcceleration = elevatorConfig.MotionMagic.MotionMagicAcceleration;

    // Put the values on the SmartDashboard if they arent already there, and then update the config
    SmartDashboard.putNumber("TEST kP",SmartDashboard.getNumber("TEST kP", kP));
    kP = SmartDashboard.getNumber("TEST kP", kP);
    SmartDashboard.putNumber("TEST kD",SmartDashboard.getNumber("TEST kD", kD));
    kD = SmartDashboard.getNumber("TEST kD", kD);
    SmartDashboard.putNumber("TEST kI",SmartDashboard.getNumber("TEST kI", kI));
    kI = SmartDashboard.getNumber("TEST kI", kI);    
    SmartDashboard.putNumber("TEST kG",SmartDashboard.getNumber("TEST kG", kG));
    kG = SmartDashboard.getNumber("TEST kG", kG);
    SmartDashboard.putNumber("TEST kS",SmartDashboard.getNumber("TEST kS", kS));
    kS = SmartDashboard.getNumber("TEST kS", kS);
    SmartDashboard.putNumber("TEST kV",SmartDashboard.getNumber("TEST kV", kV));
    kV = SmartDashboard.getNumber("TEST kV", kV);
    SmartDashboard.putNumber("TEST kA",SmartDashboard.getNumber("TEST kA", kA));
    kA = SmartDashboard.getNumber("TEST kA", kA);

    SmartDashboard.putNumber("TEST Vel",SmartDashboard.getNumber("TEST Vel", motionMagicVelocity));
    motionMagicVelocity = SmartDashboard.getNumber("TEST Vel", motionMagicVelocity);
    SmartDashboard.putNumber("TEST Accel",SmartDashboard.getNumber("TEST Accel", motionMagicAcceleration));
    motionMagicAcceleration = SmartDashboard.getNumber("TEST Accel", motionMagicAcceleration);

    elevatorConfig.Slot0.kP = kP;
    elevatorConfig.Slot0.kD = kD;
    elevatorConfig.Slot0.kI = kI;
    elevatorConfig.Slot0.kG = kG;
    elevatorConfig.Slot0.kS = kS;
    elevatorConfig.Slot0.kV = kV;
    elevatorConfig.Slot0.kA = kA;
    elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity;
    elevatorConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration;

    mElevatorLeft.getConfigurator().apply(elevatorConfig);
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
    //TalonFXConfiguration elevatorConfigRight = elevatorConfig;
    //elevatorConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //mElevatorRight.getConfigurator().apply(elevatorConfig);
  }
  
  public void SD_MotionMagicEEPivotTEST(){
    SmartDashboard.putNumber("EEPivotTestTarget", SmartDashboard.getNumber("EEPivotTestTarget", mEndeffectorPivot.getPosition().getValueAsDouble()));
    double EEPivotTestTarget = SmartDashboard.getNumber("EEPivotTestTarget", mEndeffectorPivot.getPosition().getValueAsDouble());
    mEndeffectorPivot.setControl(new MotionMagicVoltage(EEPivotTestTarget));
  }

  public void updateEEPivotConfigsFromSD(){
    // Get the current PID values and motion magic parameters
    TalonFXConfiguration EEPivotConfig = Constants.SuperstructureConfigs.getEndeffectorPivotConfig();
    mEndeffectorPivot.getConfigurator().refresh(EEPivotConfig);

    double kP = EEPivotConfig.Slot0.kP;
    double kD = EEPivotConfig.Slot0.kD;
    double kI = EEPivotConfig.Slot0.kI;
    double kG = EEPivotConfig.Slot0.kG;
    double motionMagicVelocity = EEPivotConfig.MotionMagic.MotionMagicCruiseVelocity;
    double motionMagicAcceleration = EEPivotConfig.MotionMagic.MotionMagicAcceleration;

    // Put the values on the SmartDashboard if they arent already there, and then update the config
    SmartDashboard.putNumber("TEST kP",SmartDashboard.getNumber("TEST kP", kP));
    kP = SmartDashboard.getNumber("TEST kP", kP);
    SmartDashboard.putNumber("TEST kD",SmartDashboard.getNumber("TEST kD", kD));
    kD = SmartDashboard.getNumber("TEST kD", kD);
    SmartDashboard.putNumber("TEST kI",SmartDashboard.getNumber("TEST kI", kI));
    kI = SmartDashboard.getNumber("TEST kI", kI);    
    SmartDashboard.putNumber("TEST kG",SmartDashboard.getNumber("TEST kG", kG));
    kP = SmartDashboard.getNumber("TEST kG", kG);
    SmartDashboard.putNumber("TEST Vel",SmartDashboard.getNumber("TEST Vel", motionMagicVelocity));
    motionMagicVelocity = SmartDashboard.getNumber("TEST Vel", motionMagicVelocity);
    SmartDashboard.putNumber("TEST Accel",SmartDashboard.getNumber("TEST Accel", motionMagicAcceleration));
    motionMagicAcceleration = SmartDashboard.getNumber("TEST Accel", motionMagicAcceleration);

    EEPivotConfig.Slot0.kP = kP;
    EEPivotConfig.Slot0.kD = kD;
    EEPivotConfig.Slot0.kI = kI;
    EEPivotConfig.Slot0.kG = kG;
    EEPivotConfig.MotionMagic.MotionMagicCruiseVelocity = motionMagicVelocity;
    EEPivotConfig.MotionMagic.MotionMagicAcceleration = motionMagicAcceleration;

    mEndeffectorPivot.getConfigurator().apply(EEPivotConfig);
  }

  public void manualUpdateTargets(double ElevatorPosTarget, double PivotPosTarget, double IntakePosTarget){
   elevatorTarget = ElevatorPosTarget;
   pivotTarget = PivotPosTarget;
   intakeTarget = IntakePosTarget;
  }

  /**
   * Start the intaking sequence. Sets subsystem flags to safely handle the motion to intaking
   */
  public void intake(){
  intakeTraverse();}

  public void intakeTraverse(){
    intakeTraversing = true;
    /* This block checks that the system is in a safe spot before it commands motion, and if there's potential interference(i.e. endeffector would crash) then it goes to a safe stow position first until its below a safety threshold */
    if(elevatorPos > Constants.crossbarClearancePos){ //If elevator is above the crossbar
        //if(mIntakePivot.getPosition().getValueAsDouble() < Constants.intakeEndeffectorClearancePos){ //If intake is retracted (i.e. endeffector would hit it) 
        //  TARGETSTATE = STATE.StowClear;//put the intake down and bring the elevator down with the endeffector towards the scoring side of the robot 
        //}else{
          TARGETSTATE = STATE.StowClearIntakeDeployed; //put the intake down and bring the elevator down with the endeffector towards the scoring side of the robot
        //}
    }
    else { //if the elevator is below the crossbar (i.e. wont crash the endeffector into the crossbar)    
        if(true)//intakePos < Constants.intakeEndeffectorClearancePos){ //If intake is retracted (i.e. endeffector would hit it) 
          TARGETSTATE = STATE.Intake;//put the intake down and bring the elevator down with the endeffector towards the scoring side of the robot 
        }//else {
          //TARGETSTATE = STATE.Intake; //If the elevator is below the crossbar and the endeffector will clear the intake, go to full intake
          intakeTraversing = false;
          intaking = true;
        
    
  }

  

  public void intaking(){
    releasingCoral = false;
    //hasCoral = true; //temporary for testing 2/19/2025
    if(atPosition()){
      setEndeffectorWheelSpeed(3.);
      setIntakeWheelSpeed(18); // was 13
      setFunnelWheelSpeed(-10);}//was -10

    //if (mEndeffectorRollers.getSupplyCurrent().getValueAsDouble() > 2 && CANdi.getS1State(true).getValueAsDouble() == 1 ){ //was 19 amps
    if ( CANdi.getS1State(true).getValueAsDouble() == 1 && manualOverride == false){ //was 19 amps 
      //added manualOverride boolean to allow for manual control of the intake
        //setEndeffectorHold();
      hasCoral = true;
      justGotCoral();
      setIntakeWheelSpeed(0);
      setFunnelWheelSpeed(0);
      intaking = false;
      stowing = true;
    }
  }

  public void stayIntaking(){
    intaking = true;
    setEndeffectorWheelSpeed(2);
    setIntakeWheelSpeed(13);
    setFunnelWheelSpeed(-10);
  }

  public void setEndeffectorWheelSpeed(double wheelSpeed){
    //mEndeffectorRollers.setControl(new VelocityVoltage(wheelSpeed));
    mEndeffectorRollers.setControl(new VoltageOut(-wheelSpeed));
    if(RobotBase.isSimulation()) m_EndeffectorRollers.setAngle(m_EndeffectorRollers.getAngle()+10);
  }


  public void setEndeffectorHold(){
    mEndeffectorRollers.setControl(new VoltageOut(-.5)); //stall, but we can decide to change this to PID position hold
  }

  public void setIntakeWheelSpeed(double wheelSpeed){
    
    if(RobotBase.isSimulation()) m_IntakeWheels.setAngle(m_IntakeWheels.getAngle()+10);
  
    if(wheelSpeed == 0) {mIntakeWheels.setControl(new VoltageOut(0));}
    else{
      mIntakeWheels.setControl(new VelocityVoltage(wheelSpeed));
    }}

  public void setFunnelWheelSpeed(double wheelSpeed){
    mFunnelWheels.setControl(new VoltageOut(wheelSpeed));
    if(RobotBase.isSimulation()) m_FunnelWheels.setAngle(m_FunnelWheels.getAngle()+10);
  }


  /**
   * Checks if the system is in a safe position to go to the stow position
   * @return
   */
  public Boolean safeToStow(){
    if(elevatorPos > Constants.crossbarClearancePos){// || intakePos < Constants.intakeEndeffectorClearancePos){ //if the elevator is above the crossbar or the intake is retracted
      return false;
    }
    else return true;
  }

  /**
   * Checks if the system is in a safe position to lift the Elevator
   * @return
   */
  public Boolean safeToLift(){
    if(pivotPos > Constants.endeffectorElevatorClearancePos){
      return true;
    }
    else return false;
  }

  public void clearMotionStates(){
    stowing = false;
    intakeTraversing = false;
    lifting = false;
    intaking = false;
    releasingCoral = false;
  }

  public void stopAllWheels(){
    setEndeffectorWheelSpeed(0);
    setIntakeWheelSpeed(0);
    setFunnelWheelSpeed(0);
  }

  public void goHome(){
    clearMotionStates();
    stopAllWheels();
    stowing = true;
    if(safeToStow()){
      TARGETSTATE = STATE.Home;
      stowing = false;
    }
    else TARGETSTATE = STATE.StowEEClear;
  }

  public void stow(){ //pulls the intake in and elevator down
    if(hasCoral && safeToStow()){
      // elevatorTarget = 0; //reference constants or arm pos file or replace these with states
      // pivotTarget = 0;
      TARGETSTATE = STATE.StowWithCoral;
      stowing = false;
      // STATE.StowCoral; //option for putting positions in a state
    }
    else if (hasAlgae && safeToStow()){
      // elevatorTarget = 0; //reference constants or arm pos file 
      // pivotTarget = 0;
      TARGETSTATE = STATE.StowWithAlgae;
      stowing = false;
    }
    else{
      // elevatorTarget = 0; //reference constants or arm pos file 
      // pivotTarget = 0;
      TARGETSTATE = STATE.StowEEClear;
      stowing = false;

    }
  }

  public void clearAlgae(Double level){
    if (level == 2) TARGETSTATE = STATE.grabAlgaeL2;
    if (level == 3) TARGETSTATE = STATE.grabAlgaeL3;
    setEndeffectorWheelSpeed(-5);
  }

  /**
   * Sets the elevator and endeffector pivot to the pre-score position for the coral
   * @param level
   */
  public void setPreScoreCoralState(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}

  public void setScoreCoralState(Double level){ //sets up elevator and EE pivot for score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;
    if (level == 1) TARGETSTATE = STATE.CoralL1;
    if (level == 2) TARGETSTATE = STATE.CoralL2;
    if (level == 3) TARGETSTATE = STATE.CoralL3;
    if (level == 4) TARGETSTATE = STATE.CoralL4;
  }

  public void setPostScoreCoralState(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    // elevatorTarget = 0; //reference constants or arm pos file 
    // pivotTarget = 0;    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}


  public void setCoralLevel(Double level){
    scoreLevel = level;
    scoringCoral = true;
    sequenceState = 0;
  }

  public void justGotCoral(){
    hasCoral = true;
    seatingCoral = true;
    mEndeffectorRollers.setControl(new PositionVoltage(0.0));
    mEndeffectorRollers.setPosition(3.0);
  }

	public void moveCoralIn(){
		mEndeffectorRollers.setPosition(mEndeffectorRollers.getPosition().getValueAsDouble() + 0.1); //was 0.5
	}

	public void moveCoralOut(){
		mEndeffectorRollers.setPosition(mEndeffectorRollers.getPosition().getValueAsDouble() - 0.1); //was 0.5
	}

  public void releaseCoral(){
    releasingCoral = true;
    if(scoreLevel == 1) {
      mEndeffectorRollers.setControl(new MotionMagicVelocityVoltage(-3)); //set wheelspeed here for EE rollers to release coral with the right velocity for L1
      if(mEndeffectorRollers.getPosition().getValueAsDouble() < -8) {
        hasCoral = false;
        releasingCoral = false;
        setEndeffectorWheelSpeed(0);
    }}
    else{
      mEndeffectorRollers.setControl(new PositionVoltage(10));
      if(mEndeffectorRollers.getPosition().getValueAsDouble() > 8) {
        hasCoral = false;
        releasingCoral = false;
        setEndeffectorWheelSpeed(0);
    }
  }}


  public void ureleaseCoral(){
    releasingCoral = false;
    mEndeffectorRollers.setControl(new PositionVoltage(0));
   
  }
  
  public void spit(){
    setIntakeWheelSpeed(-10);
    setFunnelWheelSpeed(10);
  }

  /**
   * releases coral with a timer to clear the 'has coral' flag and trigger the retraction
   */
  public void testReleaseCoral(){
    mEndeffectorRollers.setControl(new VoltageOut(4));
    releaseTimer++;
    if(releaseTimer > 80){//} && mEndeffectorRollers.getSupplyCurrent().getValueAsDouble() < 7){
      releaseTimer = 0;
      hasCoral = false;
    }
  }

  /**
   * pulls coral back in
   */
  public void testUnreleaseCoral(){
    releasingCoral = false;
    releaseTimer = 0;
    mEndeffectorRollers.setControl(new VoltageOut(-2));
  }

  /**
   * Starts the lifting sequence. Sets subsystem flags to safely handle the motion for lifting
   */
  public void startLifting(){
    clearMotionStates();
    lifting = true;
    sequenceState = 0;
  }

  /**
   * Lifts the elevator safely and sequentially so that the endeffector doesn't make contact with the crossbar or the reef during the scoring cycle
   */
  private void lift(){
    lifting = true;
    
    if(safeToLift()){
      if(scoringCoral){
        if(sequenceState == 0){
          setPreScoreCoralState(scoreLevel);
          if (atPosition()) {
            sequenceState = 1; //if at the pre-score position, move to the score position
          }
        }
        else if(sequenceState == 1){
          setScoreCoralState(scoreLevel);
          //if (atPosition() && autoReleaseCoral) {
          //releaseCoral();
          //sequenceState = 2;
          //}
          if(!hasCoral)sequenceState = 2;
          //sequenceState = 2;
        }
        else if(sequenceState == 2){
          setPostScoreCoralState(scoreLevel);
          if(atPosition()){
          goHome();
          sequenceState = 0;
        }
        }
      }
    }
    else TARGETSTATE = STATE.StowEEClear; //stow pre-lift
  }

  private void updatePositions(){
    if(RobotBase.isSimulation()){
      elevatorPos = m_ElevatorLeft.getLength();
      pivotPos = -180 - m_EndeffectorPivot.getAngle();
      intakePos = 60 -m_IntakeLeftPivot.getAngle();
      return;
    }
    elevatorPos = mElevatorLeft.getPosition().getValueAsDouble();
    pivotPos = mEndeffectorPivot.getPosition().getValueAsDouble() ;
    //intakePos = 100;//Temporary override 
    intakePos = mIntakePivotLeft.getPosition().getValueAsDouble() ;
  }

  public void updateSD(){
    //PID position device targets and positions
    SmartDashboard.putNumber("zIntake Pivot Target", TARGETSTATE.intake);
    SmartDashboard.putNumber("zIntake Pivot Position", intakePos);
    
    SmartDashboard.putNumber("zElevator Target", TARGETSTATE.elevator);
    SmartDashboard.putNumber("zElevator Position", elevatorPos);

    SmartDashboard.putNumber("zEndeffector Pivot Target", TARGETSTATE.pivot);
    SmartDashboard.putNumber("zEndeffector Pivot Position", pivotPos);

    //Wheelspeeds
    SmartDashboard.putNumber("zFunnel Wheelspeed", mFunnelWheels.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("zEndeffector Wheelspeed", mEndeffectorRollers.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("zIntake Wheelspeed", mIntakeWheels.getVelocity().getValueAsDouble());

    //Bools and sequences
    
    SmartDashboard.putBoolean("zAtPos", atPosition());
    SmartDashboard.putBoolean("zstowing", stowing);
    SmartDashboard.putBoolean("zintakeTraversing", intakeTraversing);
    SmartDashboard.putBoolean("zintaking", intaking);
    SmartDashboard.putBoolean("zlifting", lifting);
    SmartDashboard.putBoolean("zscoringCoral", scoringCoral);
    SmartDashboard.putBoolean("zhasCoral", hasCoral);
    
    SmartDashboard.putBoolean("zSafeToLift", safeToLift());
    SmartDashboard.putBoolean("zSafeToStow", safeToStow());
    SmartDashboard.putNumber("zsequenceState", sequenceState);
  }

  public void enableManualOverride(){
    manualOverride = true;
  }

  public void disableManualOverride(){
    manualOverride = false;
  }

  /**
   * Checks if the system is at the position within a tolerance band set in constants
   * @return
   */
  public boolean atPosition(){
    if( Math.abs(elevatorPos - TARGETSTATE.elevator) < Constants.positionTolerance && Math.abs(pivotPos - TARGETSTATE.pivot) < Constants.positionTolerance){
      return true;
    }
    else return false;
  }

  /**
   * Checks if the system is at the scoring position within a tolerance band set in constants
   * @return
   */
  public boolean atPositionScoring(){
    if( Math.abs(elevatorPos - TARGETSTATE.elevator) < Constants.scoringPositionTolerance && Math.abs(pivotPos - TARGETSTATE.pivot) < Constants.scoringPositionTolerance){
      return true;
    }
    else return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //SD_MotionMagicEEPivotTEST();
    //SD_motionMagicElevatorTEST();

    updatePositions();
    motionMagicSetElevatorAndEndeffector(TARGETSTATE.elevator, TARGETSTATE.pivot , TARGETSTATE.intake);
    
    if(intakeTraversing)intakeTraverse();
    if(intaking)intaking();
    if(stowing)stow();
    if(lifting)lift();
    if(releasingCoral)releaseCoral();

    updateSD();
    //MECH2d(); // Update the MECH2d ligaments in the periodic method
    //
    
  }

  @Override
  public void simulationPeriodic() {
    
    motionMagicSetElevatorAndEndeffector(TARGETSTATE.elevator, TARGETSTATE.pivot * Constants.endEffectorPivotGearRatio , TARGETSTATE.intake);

    if(intakeTraversing)intakeTraverse();
    if(intaking)intaking();
    if(stowing)stow();
    if(lifting)lift();

    updateSD();
    MECH2d();
    //simulateMotorMotionFeedback();
}

private void simulateMotorMotionFeedback() {
    // Simulate the encoder values for the motors
    /*if(simulatedMotorMotionTimer > 50){
      mElevatorLeft.setPosition(TARGETSTATE.elevator);
      mEndeffectorPivot.setPosition(TARGETSTATE.pivot);
      mEndeffectorPivot.setPosition(TARGETSTATE.intake);
      simulatedMotorMotionTimer = 0;}
    else simulatedMotorMotionTimer++;*/
}

  public void MECH2d(){
    SmartDashboard.putData("MyMechanism", mech);
    SmartDashboard.putNumber("ElevatorLeft", elevatorPos);
//    SmartDashboard.putNumber("ElevatorRight", mElevatorRight.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("EndeffectorPivot", pivotPos);
    SmartDashboard.putNumber("IntakePivot", intakePos);
    SmartDashboard.putNumber("EndeffectorRollers", mEndeffectorRollers.getPosition().getValueAsDouble()); 
    SmartDashboard.putNumber("IntakeWheels", mIntakeWheels.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("FunnelWheels", mFunnelWheels.getPosition().getValueAsDouble());


    // Update the lengths and angles of the ligaments based on the motor positions
    m_ElevatorLeft.setLength(m_ElevatorLeft.getLength()*.9 + TARGETSTATE.elevator*.1);
    //m_ElevatorRight.setLength(mElevatorRight.getPosition().getValueAsDouble());
    m_EndeffectorPivot.setAngle(m_EndeffectorPivot.getAngle()*.9+ (-TARGETSTATE.pivot-180) *.1);
    //m_IntakeRightPivot.setAngle(mIntakePivot.getPosition().getValueAsDouble());
    m_IntakeLeftPivot.setAngle(m_IntakeLeftPivot.getAngle()*.9 + (-TARGETSTATE.intake+60)*.1);
    m_EndeffectorRollers.setAngle(mEndeffectorRollers.getPosition().getValueAsDouble());
    m_IntakeWheels.setAngle(mIntakeWheels.getPosition().getValueAsDouble());
    m_FunnelWheels.setAngle(mFunnelWheels.getPosition().getValueAsDouble());
  }

}
