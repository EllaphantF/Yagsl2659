// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;

public class SuperstructureSubsystem extends SubsystemBase {

  private double simulatedMotorMotionTimer = 0;
  public double elevatorTarget = 0;
  public double pivotTarget = 0;
  public double intakeTarget = 0;
  public double climbPos = 0;  
  public double elevatorPos = 0;
  public double armPivotPos = 0;
  public double endeffectorPivotPos = 0;
  public double releaseTimer = 0;
  public final double intake = -20;
  public double sequenceState = 0;
  public double scoreLevel = 3;
  

  
  
  ;
  public double algaeLevel = 0;
  public boolean hasCoral = true;
  public BooleanSupplier notHasCoralCheck = () -> !hasCoral;
  public boolean hasAlgae = false;
  public boolean seatingCoral = false;
  public boolean grabbingAlgae = false;
  public boolean releasingAlgae = false;
  public double algaeTimestamp = 0;
  
  
  public static boolean autoReleaseCoral = true;

  public boolean intakeTraversing = false;
  public boolean intaking = false;
  public boolean stowing = false;
  public boolean lifting = false;
  public boolean scoringCoral = false;
  public boolean releasingCoral = false;
  public boolean releaseAtPos = false;
  public boolean manualOverride = false;

  public SuperstructureStates STATE = new SuperstructureStates();
  public SuperstructureState CURRENTSTATE = STATE.StartingConfig;
  public SuperstructureState PREVIOUSSTATE;
  public boolean previousOverrideStatus = false;
  public SuperstructureState TARGETSTATE = STATE.Home;
  
  //private final LEDs mLED = new LEDs();
/*
  private static final TalonFX mElevatorRight = new TalonFX(Constants.elevatorRightID, "rio");//MPF Hi I have IDs
  private static final TalonFX mElevatorLeft = new TalonFX(Constants.elevatorLeftID,"rio");//
  private static final TalonFX mEndeffectorPivot = new TalonFX(Constants.endEffectorPivotID,"rio");//
  private static final TalonFX mEndeffectorRollers = new TalonFX(Constants.endEffectorWheelID,"rio");//
  private static final TalonFX mIntakePivotLeft = new TalonFX(Constants.intakePivotLeftID,"Superstructure");//
  private static final TalonFX mIntakePivotRight = new TalonFX(Constants.intakePivotRightID,"Superstructure");
  private static final TalonFX mIntakeWheels = new TalonFX(Constants.intakeWheelsID,"Superstructure");//
  private static final TalonFX mFunnelWheels = new TalonFX(Constants.funnelWheelsID);//on rio loop
  private static final CANdi CANdi = new CANdi(25,"Superstructure");*/


  private static final TalonFX mElevatorRight = new TalonFX(Constants.elevatorRightID, "rio");//MPF Hi I have IDs
  private static final TalonFX mElevatorLeft = new TalonFX(Constants.elevatorLeftID,"rio");//
  private static final TalonFX mEndeffectorPivot = new TalonFX(Constants.endEffectorPivotID,"rio");//
  private static final TalonFX mEndEffectorRollersL = new TalonFX(Constants.eeWheelsLeftID, "rio");//
  private static final TalonFX mArmPivot = new TalonFX(Constants.armID,"rio");//
  private static final TalonFX mEndEffectorRollersR = new TalonFX(Constants.eeWheelsRightID,"rio");
  private static final TalonFX mFunnelWheelsTop = new TalonFX(Constants.topFunnelWheelsID,"rio");//
  private static final TalonFX mFunnelWheelsBottom = new TalonFX(Constants.bottomFunnelWheelsID,"rio");//
  private static final TalonFX mClimbPivot = new TalonFX(Constants.climbPivotID,"rio");
  private static final CANdi CANdi = new CANdi(0);

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

  public int lightMode = 0; 
  /* disabled = 0, 
   * autonomous = 1
   * no coral = 2
   * holding coral = 3
   * scoring L1 = 4
   * scoring L2 = 5
   * scoring L3 = 6
   * scoring L4 = 7
   * intaking = 8
   * climbing = 9
   * algae = 10
   * homing = 11
   * releasing = 12
   * at position = 13
   */

  /** Creates a new ExampleSubsystem. */
  public SuperstructureSubsystem() {
        
    /* Configurations */
        
        mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeft());
        //mElevatorRight.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigRight());
        
        mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true)); //try removing this

        mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig());
        mEndEffectorRollersL.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorWheelsConfigurationLeft());//
        mEndEffectorRollersR.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorWheelsConfiguration()); //

        mArmPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getArmPivotConfiguration()); //
        mFunnelWheelsBottom.getConfigurator().apply(Constants.SuperstructureConfigs.getFunnelWheelsConfiguration());
        mFunnelWheelsTop.getConfigurator().apply(Constants.SuperstructureConfigs.getFunnelWheelsConfiguration());
        mClimbPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getClimbPivotConfiguration());


         /* Mechanism2d */
        
        mech = new Mechanism2d(50, 120);
        root = mech.getRoot("Center", 13.5, 13.5);
        MechanismRoot2d elevatorLeftRoot = mech.getRoot("ElevatorLeftRoot", 10, 25);
        MechanismRoot2d intakePivotLeftRoot = mech.getRoot("IntakePivotLeftRoot", 20, 5);
        m_ElevatorLeft = elevatorLeftRoot.append(new MechanismLigament2d("ElevatorLeft", 65, 90));
        m_EndeffectorPivot = m_ElevatorLeft.append(new MechanismLigament2d("EndeffectorPivot", 12, 270));
        m_EndeffectorRollers = m_ElevatorLeft.append(new MechanismLigament2d("EndeffectorRollers", 3, 90));
        m_IntakeLeftPivot = intakePivotLeftRoot.append(new MechanismLigament2d("IntakePivot", 15, 90));
        m_IntakeWheels = m_IntakeLeftPivot.append(new MechanismLigament2d("IntakeWheels", 3, 90));
        m_FunnelWheels = root.append(new MechanismLigament2d("FunnelWheels", 3, 90));

  }

  public void setCurrentState(){
    
    CURRENTSTATE.arm = armPivotPos;
    CURRENTSTATE.elevator = elevatorPos;
    CURRENTSTATE.climb = climbPos;
    CURRENTSTATE.ee = endeffectorPivotPos;

    TARGETSTATE = CURRENTSTATE;
  }

  public  void climb(double state){
    
    if (state == 1) {
      TARGETSTATE = STATE.climb1;
      // mLED.setLightMode(1);
      lightMode = 9;
    }
    if (state == 2) {
      TARGETSTATE = STATE.climb2; 
      setIntakeWheelSpeed(3);
      // mLED.setLightMode(1);
      lightMode = 9;
    }
    if (state == 3) {
      TARGETSTATE = STATE.climb3;
//      mIntakePivotLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakePivotLeftConfigurationCLIMB()); //
//      mIntakePivotRight.getConfigurator().apply(Constants.SuperstructureConfigs.getIntakePivotRightConfigurationCLIMB()); //
      // mLED.setLightMode(2);
      lightMode = 9;
    }
  }

  public void motionMagicSetElevatorAndEndeffector(double ElevatorPosTarget, double ArmPivotPosTarget, double climbPosTarget, double EndeffectorPivotTarget){
    mElevatorLeft.setControl(new MotionMagicVoltage(ElevatorPosTarget)); //elevator right is following left
    //mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
    mEndeffectorPivot.setControl(new MotionMagicVoltage(EndeffectorPivotTarget ));

    mArmPivot.setControl(new MotionMagicVoltage(ArmPivotPosTarget));


  }

  public void SD_motionMagicElevatorTEST(){
    SmartDashboard.putNumber("ElevatorTestTarget", SmartDashboard.getNumber("ElevatorTestTarget", mElevatorLeft.getPosition().getValueAsDouble()));
    double ElevatorTestTarget = SmartDashboard.getNumber("ElevatorTestTarget", mElevatorLeft.getPosition().getValueAsDouble());
    mElevatorLeft.setControl(new MotionMagicVoltage(ElevatorTestTarget));
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));
  }

  /*
   * Sets target state to intake
   * 
   *    */
  public void nomNomWeebleWobble (){

    //SmartDashboard.putBoolean("zzzdebugNOMNOM", Timer.getFPGATimestamp()%.6 > .5);
    /*if (Timer.getFPGATimestamp() %.6 > .5){
      TARGETSTATE = STATE.IntakeWobble;
    } 
    else */ TARGETSTATE = STATE.Intake;

  }

  public void updateElevatorConfigsFromSD(){
    // Get the current PID values and motion magic parameters
    /*TalonFXConfiguration elevatorConfig = Constants.SuperstructureConfigs.getElevatorConfigLeft();
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
    mElevatorRight.setControl(new Follower(mElevatorLeft.getDeviceID(), true));*/
  }
  
  public void SD_MotionMagicEEPivotTEST(){
    /*SmartDashboard.putNumber("EEPivotTestTarget", SmartDashboard.getNumber("EEPivotTestTarget", mEndeffectorPivot.getPosition().getValueAsDouble()));
    double EEPivotTestTarget = SmartDashboard.getNumber("EEPivotTestTarget", mEndeffectorPivot.getPosition().getValueAsDouble());
    mEndeffectorPivot.setControl(new MotionMagicVoltage(EEPivotTestTarget));*/
  }

  public void updateEEPivotConfigsFromSD(){
    // Get the current PID values and motion magic parameters
    /*TalonFXConfiguration EEPivotConfig = Constants.SuperstructureConfigs.getEndeffectorPivotConfig();
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

    mEndeffectorPivot.getConfigurator().apply(EEPivotConfig);*/
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
    mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeftCoral());
    mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfigCoral());
    mArmPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getArmPivotConfigurationCoral());
        
  intakeTraverse();}

  public void intakeTraverse(){
    intakeTraversing = true;
    /* This block checks that the system is in a safe spot before it commands motion, and if there's potential interference(i.e. endeffector would crash) then it goes to a safe stow position first until its below a safety threshold */
    if(elevatorPos > Constants.crossbarClearancePos){ //If elevator is above the crossbar
          
          lightMode = 8;
          TARGETSTATE = STATE.StowClearIntakeDeployed; //put the intake down and bring the elevator down with the endeffector towards the scoring side of the robot
        //}
    }
    else { //if the elevator is below the crossbar (i.e. wont crash the endeffector into the crossbar)    
          TARGETSTATE = STATE.Intake;//put the intake down and bring the elevator down with the endeffector towards the scoring side of the robot 
          
          lightMode = 8;
          intakeTraversing = false;
          intaking = true;
        }
    
  }

  

  public void intaking(){
    releasingCoral = false;
    //hasCoral = true; //temporary for testing 2/19/2025
    TARGETSTATE = STATE.Intake;
    lightMode = 8;
    if(atPosition()){
      lightMode = 3;
      setEndeffectorWheelSpeed(3,3);
      //setIntakeWheelSpeed(45); // was 23 -- 40 works great 3-8-2025
      setFunnelWheelSpeed(-4);}//was -10 -- -8 works great 3-8-2025
    if ( CANdi.getS1State(true).getValueAsDouble() == 1){ //was 19 amps CANDi closed
    
      justGotCoral();
      setFunnelWheelSpeed(0);
      intaking = false;
      lightMode = 6; 
      //stowing = true; //removed 3-8-25 so that coral will fully stow before EE pivot stows
    }
  }

  public void stayIntaking(){
    intaking = true;
    setEndeffectorWheelSpeed(4,-4);
    setIntakeWheelSpeed(13);
    setFunnelWheelSpeed(-10);
  }

/**
 *use Left/Right differential to spin-spit for L1 
 *for both wheels, positive is forwards, for feeding and for out-taking/scoring 
 * @param wheelSpeedL
 * @param wheelSpeedR
 */
  public void setEndeffectorWheelSpeed(double wheelSpeedL,double wheelSpeedR){
    //mEndeffectorRollers.setControl(new VelocityVoltage(wheelSpeed));
    mEndEffectorRollersL.setControl(new VoltageOut(wheelSpeedL));
    mEndEffectorRollersR.setControl(new VoltageOut(wheelSpeedR));
    if(RobotBase.isSimulation()) m_EndeffectorRollers.setAngle(m_EndeffectorRollers.getAngle()+10);
  }

  public void setEndeffectorWheelVelocity(double wheelSpeedL,double wheelSpeedR){
    //mEndeffectorRollers.setControl(new VelocityVoltage(wheelSpeed));
    mEndEffectorRollersL.setControl(new MotionMagicVelocityVoltage(wheelSpeedL));
    mEndEffectorRollersR.setControl(new MotionMagicVelocityVoltage(wheelSpeedR));
    if(RobotBase.isSimulation()) m_EndeffectorRollers.setAngle(m_EndeffectorRollers.getAngle()+10);
  }


  public void setEndeffectorHold(){
    //mEndeffectorRollers.setControl(new VoltageOut(-.5)); //stall, but we can decide to change this to PID position hold
  }

  public void setIntakeWheelSpeed(double wheelSpeed){
    
  /*  if(RobotBase.isSimulation()) m_IntakeWheels.setAngle(m_IntakeWheels.getAngle()+10);
  
    if(wheelSpeed == 0) {mIntakeWheels.setControl(new VoltageOut(0));}
    else{
      mIntakeWheels.setControl(new VelocityVoltage(-wheelSpeed));
  }*/ }

    /**
     * set voltage output for feeder wheels
     * @param wheelSpeed
     */
  public void setFunnelWheelSpeed(double wheelSpeed){
    mFunnelWheelsTop.setControl(new VoltageOut(-wheelSpeed));
    mFunnelWheelsBottom.setControl(new VoltageOut(-wheelSpeed*0.8));
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
    if(armPivotPos > Constants.endeffectorElevatorClearancePos){
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
    seatingCoral = false;
    grabbingAlgae = false;
    releasingAlgae = false;
  }

  public void stopAllWheels(){
    setFunnelWheelSpeed(0);
    if(hasAlgae) setEndeffectorWheelSpeed(-2,-2);
    else setEndeffectorWheelSpeed(0,0);
  }

  public void goHome(){
    // mLED.setLightMode(1);
    lightMode = 11; 
    clearMotionStates();
    stopAllWheels();
    stowing = true;


    if(safeToStow()){
      //mLED.setLightMode(8);
      TARGETSTATE = STATE.Home;
      stowing = false;
      setIntakeWheelSpeed(2);

      if(hasCoral == true){
        lightMode = 3;
      } else {
        lightMode = 2;
      }
    }
    else TARGETSTATE = STATE.StowEEClear;

    if(hasCoral == true){
      lightMode = 3;
    } else {
      lightMode = 2;
    }
  }



  private void stow(){ //pulls the intake in and elevator down
    if(hasCoral && safeToStow()){
      if (scoreLevel == 3 || scoreLevel == 4) TARGETSTATE = STATE.StowPreL34;//StowWithCoral
      else TARGETSTATE = STATE.StowWithCoral;
      stowing = false;
      // STATE.StowCoral; //option for putting positions in a state  

    }

    
    else if (TARGETSTATE == STATE.bargeAlgae)
      TARGETSTATE = STATE.StowPreL34;

    else if (hasAlgae && safeToStow()){
      TARGETSTATE = STATE.StowWithAlgae;
      stowing = false;
    }

    else{
      TARGETSTATE = STATE.StowEEClear;
      stowing = false;

    }
  }

  /**
 * was previously 'clear algae'
 * @param level
 */
public void groundIntakeAlgae(){
  
  mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeft());
  mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig());
  mArmPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getArmPivotConfiguration());

  grabbingAlgae = true;
  TARGETSTATE = STATE.groundIntakeAlgae;
  setEndeffectorWheelVelocity(-90,-90);
  //setEndeffectorWheelSpeed(-10, -10);
  lightMode = 10;
}

/**
 * was previously 'clear algae'
 * @param level
 */
  public void grabAlgae(Double level){
    
  mElevatorLeft.getConfigurator().apply(Constants.SuperstructureConfigs.getElevatorConfigLeft());
  mEndeffectorPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getEndeffectorPivotConfig());
  mArmPivot.getConfigurator().apply(Constants.SuperstructureConfigs.getArmPivotConfiguration());

    grabbingAlgae = true;
    if (level == 2) TARGETSTATE = STATE.grabAlgaeL2;
    if (level == 3) TARGETSTATE = STATE.grabAlgaeL3;
    setEndeffectorWheelVelocity(-60, -60); //was Speed -10
    //setEndeffectorWheelSpeed(-10, -10);
    lightMode = 10;
  }

  public void grabbingAlgae(){
    
    if(CANdi.getS1State(true).getValueAsDouble() == 1){ //CANDi closed
      setEndeffectorWheelSpeed(-2.5,-2.5); //ACE - tune the holding voltage here
      hasAlgae = true;
      grabbingAlgae = false;
      if(TARGETSTATE == STATE.grabAlgaeL2) TARGETSTATE = STATE.StowWithAlgaeL2;
      else if(TARGETSTATE == STATE.grabAlgaeL3) TARGETSTATE = STATE.StowWithAlgaeL3;
      else TARGETSTATE = STATE.StowWithAlgae;
    }
  }

  public void panic(){
    TARGETSTATE = STATE.CoralL4;
  }

  /**
   * Sets the elevator and endeffector pivot to the pre-score position for the coral
   * @param level
   */
  public void setPreScoreCoralState(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up 
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;}

  public void setScoreCoralState(Double level){ //sets up elevator and EE pivot for score position for drive-up 
    if (level == 1) TARGETSTATE = STATE.CoralL1;
    if (level == 2) TARGETSTATE = STATE.CoralL2;
    if (level == 3) TARGETSTATE = STATE.CoralL3;
    if (level == 4) TARGETSTATE = STATE.CoralL4;
  }

  public void setPostScoreCoralState(Double level){ //sets up elevator and EE pivot for pre-score position for drive-up    
    if (level == 1) TARGETSTATE = STATE.CoralPreL1;
    if (level == 2) TARGETSTATE = STATE.CoralPreL2;
    if (level == 3) TARGETSTATE = STATE.CoralPreL3;
    if (level == 4) TARGETSTATE = STATE.CoralPreL4;
  }

  public void setCoralLevel(Double level){
    scoreLevel = level;
    scoringCoral = true;
    sequenceState = 0; //2659
  }
  

  public void justGotCoral(){
    if(hasCoral && mArmPivot.getPosition().getValueAsDouble() > .5){//-5 * Constants.endEffectorPivotGearRatio / 360 ){     ACE - tune this number
      //SmartDashboard.putNumber("zEEDebug", 0.5);
      mEndEffectorRollersL.setControl(new MotionMagicVoltage(0.0));
      mEndEffectorRollersL.setPosition(0.5);
      mEndEffectorRollersR.setControl(new MotionMagicVoltage(0.0));
      mEndEffectorRollersR.setPosition(0.5);
      intaking = false;}
    else{
      //SmartDashboard.putNumber("zEEDebug", 3);
      mEndEffectorRollersL.setControl(new MotionMagicVoltage(0.0));
      mEndEffectorRollersL.setPosition(.5); //3 for wheelspeed 4 (inconsistent), 4 for wheelspeed 2
      mEndEffectorRollersR.setControl(new MotionMagicVoltage(0.0));
      mEndEffectorRollersR.setPosition(.5); //3 for wheelspeed 4 (inconsistent), 4 for wheelspeed 2
      intaking = false;
    }
    hasCoral = true;
    seatingCoral = true;
  }

  private void seatCoral(){
    if(Math.abs( mEndEffectorRollersL.getClosedLoopError().getValueAsDouble()) < .5){
      stowing = true;
      seatingCoral = false;
    }
  }
/*
	public void moveCoralIn(){
		mEndeffectorRollers.setPosition(mEndeffectorRollers.getPosition().getValueAsDouble() + 0.1); //was 0.5
	}

	public void moveCoralOut(){
		mEndeffectorRollers.setPosition(mEndeffectorRollers.getPosition().getValueAsDouble() - 0.1); //was 0.5
	}*/

/**
 * setting whenAtPos to true makes the superstructure wait until its at position to score
 * @param whenAtPos
 */
  public void startReleasingCoral(boolean whenAtPos){
    if (whenAtPos) {
      releaseAtPos = true;
    }
    else releaseAtPos = false;

    releaseCoral();
  }

  private void releaseCoral(){
    releasingCoral = true;
    if(!releaseAtPos || atPositionScoring()){
    // mLED.setLightMode(7);
    lightMode = 12;
    if(scoreLevel == 1) {
      setEndeffectorWheelSpeed(20,2); //sideways spin-release for L1 - ACE - tune these numbers
      if(mEndEffectorRollersL.getPosition().getValueAsDouble() > 8) {
        hasCoral = false;
        hasAlgae = false;
        releasingCoral = false;
        releaseAtPos = false;
        setEndeffectorWheelSpeed(0,0);
        lightMode = 2;
    }}
    else{
      setEndeffectorWheelSpeed(15, 15);
      if(mEndEffectorRollersL.getPosition().getValueAsDouble() > 8 &&  CANdi.getS1State(true).getValueAsDouble() == 0) {//check if beam break is open
        hasCoral = false;
        hasAlgae = false;
        releasingCoral = false;
        releaseAtPos = false;
        setEndeffectorWheelSpeed(0,0);
    }
  }}}

  public BooleanSupplier notHasCoralCheck(){
    //SmartDashboard.putNumber("notHasCoralCheck", Timer.getFPGATimestamp());
  return notHasCoralCheck;
  }

  public void goToBargeAlgaeScoring(){
    TARGETSTATE = STATE.bargeAlgae;
  }
 
  public void goToProcessorAlgaeScoring(){
    TARGETSTATE = STATE.processorAlgae;
  }

  public void releaseAlgae(double speed){
    releasingAlgae = true;
    algaeTimestamp = Timer.getFPGATimestamp();
    setEndeffectorWheelSpeed(speed, speed);
  }

  public void releasingAlgae(){
    if(CANdi.getS1State(true).getValueAsDouble() == 1 && Timer.getFPGATimestamp() - algaeTimestamp > 0.3){ //if the CANdi says it's released and it's been at least 0.3 seconds
      setEndeffectorWheelSpeed(0,0);
      hasAlgae = false;
      releasingAlgae = false;
    }   
  }

  public void spit(){
    setFunnelWheelSpeed(-2);
  }


  /**
   * unreleases coral and goes back to holding position
   */
  public void ureleaseCoral(){
    mEndEffectorRollersL.setControl(new MotionMagicVoltage(0.0));
    mEndEffectorRollersR.setControl(new MotionMagicVoltage(0.0));
    
  }

  public void elevatorHoldPos(){
    //TARGETSTATE.elevator = mElevatorLeft.getPosition().getValueAsDouble();
    //SmartDashboard.putNumber("zzz elevator hold debug", Timer.getFPGATimestamp());
  }


  public Command autoCoral(){
    InstantCommand score = new InstantCommand(() -> releaseCoral());
    return score;
  }

  /**
   * Starts the lifting sequence. Sets subsystem flags to safely handle the motion for lifting
   */
  public void startLifting(){
    clearMotionStates();
    lifting = true;
    sequenceState = 0;
    hasCoral = true; 
  }

  public void setSuperstructurePrescore(){
    clearMotionStates();
    sequenceState = 0; 
    TARGETSTATE = STATE.CoralPreL4;
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

          if(scoreLevel == 1){
            lightMode = 4;
          } else if(scoreLevel == 2){
            lightMode = 5;
          } else if(scoreLevel == 3){
            lightMode = 6;
          } else if(scoreLevel == 4){
            lightMode = 7;
          }

          if (atPosition()) {
            sequenceState = 1; //if at the pre-score position, move to the score position
          }
        }
        else if(sequenceState == 1){
          setScoreCoralState(scoreLevel);
          if(!hasCoral)sequenceState = 2;
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
      endeffectorPivotPos = -180 - m_EndeffectorPivot.getAngle();
      armPivotPos = 60 -m_IntakeLeftPivot.getAngle();
      return;
    }

    elevatorPos = mElevatorLeft.getPosition().getValueAsDouble();
    endeffectorPivotPos = mEndeffectorPivot.getPosition().getValueAsDouble();
    armPivotPos = mArmPivot.getPosition().getValueAsDouble();
    climbPos = mClimbPivot.getPosition().getValueAsDouble();

  }

  public void updateSD(){
    //PID position device targets and positions
    SmartDashboard.putNumber("Arm Pivot Target", TARGETSTATE.arm);
    SmartDashboard.putNumber("Arm Pivot Position", armPivotPos);
    
    SmartDashboard.putNumber("Elevator Target", TARGETSTATE.elevator);
    SmartDashboard.putNumber("Elevator Position", elevatorPos);

    SmartDashboard.putNumber("Endeffector Pivot Target", TARGETSTATE.ee);
    SmartDashboard.putNumber("Endeffector Pivot Position", endeffectorPivotPos);

    //Wheelspeeds
    //SmartDashboard.putNumber("zFunnel Top Wheelspeed", mFunnelWheelsTop.getVelocity().getValueAsDouble());
    //SmartDashboard.putNumber("zEndeffector Wheelspeed", mEndEffectorRollersL.getVelocity().getValueAsDouble());

    //Bools and sequences
    
    SmartDashboard.putBoolean("Superstructure At Pos", atPosition());
    SmartDashboard.putBoolean("Superstructure At Scoring Pos", atPositionScoring());
    SmartDashboard.putBoolean("Stowing", stowing);
    SmartDashboard.putBoolean("IntakeTraversing", intakeTraversing);
    SmartDashboard.putBoolean("Intaking", intaking);
    SmartDashboard.putBoolean("Lifting", lifting);
    SmartDashboard.putBoolean("ScoringCoral", scoringCoral);
    SmartDashboard.putBoolean("HasCoral", hasCoral);
    SmartDashboard.putBoolean("HasAlgae", hasAlgae);
    
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
    if( Math.abs(elevatorPos - TARGETSTATE.elevator) < Constants.positionTolerance && Math.abs(endeffectorPivotPos - TARGETSTATE.ee) < Constants.positionTolerance && Math.abs(armPivotPos - TARGETSTATE.arm) < Constants.positionTolerance){
      return true;
    }
    else return false;
  }

  /**
   * Checks if the system is at the scoring position within a tolerance band set in constants
   * @return
   */
  public boolean atPositionScoring(){
    boolean eeAtPos = false;
    boolean elevatorAtPos = false;
    boolean armAtPos = false;
      if(scoreLevel == 1){
        elevatorAtPos = Math.abs(elevatorPos - STATE.CoralL1.elevator) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Elevator At Pos Scoring", elevatorAtPos);
        eeAtPos = Math.abs(endeffectorPivotPos - STATE.CoralL1.ee) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("EE At Pos Scoring", eeAtPos);
        armAtPos = Math.abs(armPivotPos - STATE.CoralL1.arm) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Arm At Pos Scoring", armAtPos);
        if(elevatorAtPos  && eeAtPos && armAtPos){
          // mLED.setLightMode(7);
            SmartDashboard.putBoolean("At Position Scoring", true);
            lightMode = 13;
            return true; }}
      else if (scoreLevel == 2){
        elevatorAtPos = Math.abs(elevatorPos - STATE.CoralL2.elevator) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Elevator At Pos Scoring", elevatorAtPos);
        eeAtPos = Math.abs(endeffectorPivotPos - STATE.CoralL2.ee) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("EE At Pos Scoring", eeAtPos);
        armAtPos = Math.abs(armPivotPos - STATE.CoralL2.arm) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Arm At Pos Scoring", armAtPos);
        if(elevatorAtPos  && eeAtPos && armAtPos){
          // mLED.setLightMode(7);
            SmartDashboard.putBoolean("At Position Scoring", true);
            lightMode = 13;
            return true; }}
      else if(scoreLevel == 3){
        elevatorAtPos = Math.abs(elevatorPos - STATE.CoralL3.elevator) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Elevator At Pos Scoring", elevatorAtPos);
        eeAtPos = Math.abs(endeffectorPivotPos - STATE.CoralL3.ee) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("EE At Pos Scoring", eeAtPos);
        armAtPos = Math.abs(armPivotPos - STATE.CoralL3.arm) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Arm At Pos Scoring", armAtPos);
        if(elevatorAtPos  && eeAtPos && armAtPos){
          // mLED.setLightMode(7);
            SmartDashboard.putBoolean("At Position Scoring", true);
            lightMode = 13;
            return true; }}
      else if(scoreLevel == 4){
        elevatorAtPos = Math.abs(elevatorPos - STATE.CoralL4.elevator) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Elevator At Pos Scoring", elevatorAtPos);
        eeAtPos = Math.abs(endeffectorPivotPos - STATE.CoralL4.ee) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("EE At Pos Scoring", eeAtPos);
        armAtPos = Math.abs(armPivotPos - STATE.CoralL4.arm) < Constants.scoringPositionTolerance;
        SmartDashboard.putBoolean("Arm At Pos Scoring", armAtPos);
        if(elevatorAtPos  && eeAtPos && armAtPos){
          // mLED.setLightMode(7);
            SmartDashboard.putBoolean("At Position Scoring", true);
            lightMode = 13;
            return true; }}
      else {
        // mLED.setLightMode(0);
        return false;}
        SmartDashboard.putBoolean("At Position Scoring", false);
      // mLED.setLightMode(7);
      return false;
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //SD_MotionMagicEEPivotTEST();
    //SD_motionMagicElevatorTEST();
    SmartDashboard.putNumber("CANDi State", CANdi.getS1State().getValueAsDouble());
    //SmartDashboard.putNumber("Superstructure Timer Debug", Timer.getFPGATimestamp());
    
    updatePositions();

    if(intakeTraversing)intakeTraverse();
    if(intaking)intaking();
    if(stowing)stow();
    if(lifting)lift();
    if(releasingCoral)releaseCoral();
    if(seatingCoral)seatCoral();
    if(grabbingAlgae)grabbingAlgae();

    updateSD();

    if (TARGETSTATE != PREVIOUSSTATE || previousOverrideStatus != manualOverride) motionMagicSetElevatorAndEndeffector(TARGETSTATE.elevator, TARGETSTATE.arm, TARGETSTATE.climb,  TARGETSTATE.ee);
    
    PREVIOUSSTATE = TARGETSTATE;
    previousOverrideStatus = manualOverride;
    //MECH2d(); // Update the MECH2d ligaments in the periodic method
    //
    
  }

  @Override
  public void simulationPeriodic() {
    
    //motionMagicSetElevatorAndEndeffector(TARGETSTATE.elevator, TARGETSTATE.pivot * Constants.endEffectorPivotGearRatio , TARGETSTATE.intake);

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
    SmartDashboard.putNumber("EndeffectorPivot", endeffectorPivotPos);
    SmartDashboard.putNumber("ArmPivot", armPivotPos);
    SmartDashboard.putNumber("EndeffectorRollers", mEndEffectorRollersL.getPosition().getValueAsDouble()); 
    SmartDashboard.putNumber("FunnelWheels", mFunnelWheelsTop.getPosition().getValueAsDouble());


    // Update the lengths and angles of the ligaments based on the motor positions
    m_ElevatorLeft.setLength(m_ElevatorLeft.getLength()*.9 + TARGETSTATE.elevator*.1);
    //m_ElevatorRight.setLength(mElevatorRight.getPosition().getValueAsDouble());
    m_EndeffectorPivot.setAngle(m_EndeffectorPivot.getAngle()*.9+ (-TARGETSTATE.ee-180) *.1);
    //m_IntakeRightPivot.setAngle(mIntakePivot.getPosition().getValueAsDouble());
    //m_IntakeLeftPivot.setAngle(m_IntakeLeftPivot.getAngle()*.9 + (-TARGETSTATE.intake+60)*.1);
    m_EndeffectorRollers.setAngle(mEndEffectorRollersL.getPosition().getValueAsDouble());
    m_FunnelWheels.setAngle(mFunnelWheelsTop.getPosition().getValueAsDouble());
  }

}
