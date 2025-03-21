// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
//import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
//import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ProxyCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeL2Command;
import frc.robot.commands.AutonScoreCommand;
import frc.robot.commands.GoHomeCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.L1Command;
import frc.robot.commands.L2Command;
import frc.robot.commands.L3Command;
import frc.robot.commands.L4Command;
/*import frc.robot.commands.L1Command;
import frc.robot.commands.L2Command;
import frc.robot.commands.L3Command;
import frc.robot.commands.L4Command;*/
import frc.robot.commands.VisionIntakeCommand;
//import frc.robot.commands.*;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

//import drivebase.driveToPose;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */


 /* I ADDED PATH PLANNER STUFF MPF */

 
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  final         CommandJoystick buttonBox1 = new CommandJoystick(2);
  final         CommandJoystick buttonBox2 = new CommandJoystick(3);
  
  final         CommandXboxController       buttonBox = new CommandXboxController(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                //"swerve/neo"));
                                                                                "swerve/falcon"));
  private final SuperstructureSubsystem superstructure = new SuperstructureSubsystem();
      // the main mechanism object
  Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
  MechanismRoot2d root = mech.getRoot("intake", 2, 0);

  //PathPlannerAuto pathPlannerAuto = new PathPlannerAuto(getAutonomousCommand());

  private final SendableChooser<Command> autoChooser;
  public static SendableChooser<Alliance> allianceChooser;
  
  public double count = 0;

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox.getHID()::getYButtonPressed,
                                                                 driverXbox.getHID()::getAButtonPressed,
                                                                 driverXbox.getHID()::getXButtonPressed,
                                                                 driverXbox.getHID()::getBButtonPressed);

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)//
                                                          //.withControllerRotationAxis(driverXbox::getRightX)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX()*-1) //BVN 1-26-25 - added negative to reverse the rotation input, removed 2/3
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /* Command driveFieldOrientedDirectAngleTwo = drivebase.driveCommandTwo(
                                                            () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                                                            () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_Y_DEADBAND),
                                                            () -> driverXbox.getRightX(),
                                                            () -> driverXbox.getRightY()); */

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> driverXbox.getLeftY(),
                                                                   () -> driverXbox.getLeftX())
                                                               .withControllerRotationAxis(() -> driverXbox.getRawAxis(1))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    driverXbox.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);

  //Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

 // AutoScoreCommand AutoScoreCommand = new AutoScoreCommand(superstructure, drivebase);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    /* Added Named Commands for Pathplanner */
	  NamedCommands.registerCommand("Intake", new IntakeCommand(superstructure).withTimeout(3));
    NamedCommands.registerCommand("L1", new L1Command(superstructure).withTimeout(3));
    NamedCommands.registerCommand("L2", new L2Command(superstructure).withTimeout(3));
    NamedCommands.registerCommand("L3", new L3Command(superstructure).withTimeout(6));
    NamedCommands.registerCommand("L4", new L4Command(superstructure).withTimeout(6));
    NamedCommands.registerCommand("GoHome", new GoHomeCommand(superstructure));
    NamedCommands.registerCommand("AlgaeL2", new AlgaeL2Command(superstructure).withTimeout(3));
	  //NamedCommands.registerCommand("VisionIntake", new VisionIntakeCommand(this, getSuperstructure(), getSwerveSubsystem(),  5.0));
    
    NamedCommands.registerCommand("AutonScoreCommandP1L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  1 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP2L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  2 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP3L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  3 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP4L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  4 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP5L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  5 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP6L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  6 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP7L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  7 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP8L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  8 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP9L4" , new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(),  9 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP10L4", new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(), 10 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP11L4", new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(), 11 , 4));
    NamedCommands.registerCommand("AutonScoreCommandP12L4", new AutonScoreCommand(this, getSuperstructure(), getSwerveSubsystem(), 12 , 4));
    // buttonBox = new ButtonBox();
    // buttonBox = new ButtonBox();
    // Configure the trigger bindings
    configureBindings();

    /* allianceChooser = new SendableChooser<>();
    allianceChooser.setDefaultOption("Blue", Alliance.Blue);
    allianceChooser.addOption("Red", Alliance.Red);
    SmartDashboard.putData("Alliance Color", allianceChooser); */

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() //BVN Note - called from "set drive mode" method below, which is called in some init commands in robot.java
  {
    // (Condition) ? Return-On-True : Return-on-False
    drivebase.setDefaultCommand(!RobotBase.isSimulation() ?
                                //driveFieldOrientedDirectAngle :
                                driveFieldOrientedAnglularVelocity:
                                driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest()) //BVN 1-26-25 note, if we enable test itll bind these, but it doesnt unbind them if we enable tele, it just over writes
                                // so some of the button bindings are still there from enabling one mode(i.e. test), then enabling the other (i.e. tele).
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));

    } else
    {
      // driverXbox.povLeft().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", SmartDashboard.getNumber("Select Scoring Location",0)-.5)));
      // driverXbox.povRight().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", SmartDashboard.getNumber("Select Scoring Location",0)+.5)));

      /* Drive Controller */
      driverXbox.povLeft().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.povRight().onTrue((Commands.runOnce(drivebase::resetDriveEncoders)));
      driverXbox.b().whileTrue(new InstantCommand(() -> superstructure.startReleasingCoral(false)).repeatedly());
      driverXbox.b().onFalse(new InstantCommand(() -> superstructure.ureleaseCoral()));
      driverXbox.a().onTrue(new InstantCommand(() -> superstructure.disableManualOverride())); //3-4-25 MPF added disable manual override to intake
      driverXbox.leftBumper().onTrue((Commands.runOnce(drivebase::resetDriveEncoders)));
      //driverXbox.leftBumper().onTrue(new InstantCommand(() -> superstructure.intake()));
      //driverXbox.leftBumper().whileTrue(visionIntake());

      driverXbox.start().onTrue(new InstantCommand(() -> superstructure.climb(1)));
      driverXbox.back().onTrue(new InstantCommand(() -> superstructure.climb(2)));
      driverXbox.povDown().onTrue(new InstantCommand(() -> superstructure.climb(3)));

      // Bind the Xbox button to the getScoreSequenceCommand
      driverXbox.rightTrigger(.5).whileTrue(new StartEndCommand(
          () -> getScoreSequenceCommand(true).schedule(),
          () -> CommandScheduler.getInstance().cancelAll()
          ));
      driverXbox.rightTrigger(.5).onFalse(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll())); //this seems to work, but might cancel other commands? Drive seems to work fine after this is called

      /* Operator Controller **TEST** */
      /* operatorXbox.leftBumper().onTrue(new InstantCommand(() -> superstructure.setCoralLevel(1.)));
      operatorXbox.back().onTrue(new InstantCommand(() -> superstructure.setCoralLevel(2.)));
      operatorXbox.start().onTrue(new InstantCommand(() -> superstructure.setCoralLevel(3.)));
      operatorXbox.rightBumper().onTrue(new InstantCommand(() -> superstructure.setCoralLevel(4.)));
      operatorXbox.y().onTrue(new InstantCommand(() -> superstructure.startLifting()));
      operatorXbox.x().onTrue(new InstantCommand(() -> superstructure.goHome()));
      operatorXbox.a().onTrue(new InstantCommand(() -> superstructure.intake()));
      operatorXbox.b().whileTrue(new InstantCommand(() -> superstructure.startReleasingCoral(false)).repeatedly());
      operatorXbox.b().onFalse(new InstantCommand(() -> superstructure.ureleaseCoral()));
      operatorXbox.povDown().onTrue(new InstantCommand( () -> superstructure.updateElevatorConfigsFromSD()));
      operatorXbox.povRight().whileTrue(new InstantCommand( () -> superstructure.spit()).repeatedly());
      operatorXbox.povLeft().onTrue(new InstantCommand( () -> superstructure.moveCoralIn()));
      operatorXbox.povUp().onTrue(new InstantCommand( () -> superstructure.moveCoralOut())); */
      //operatorXbox.a().onTrue(Commands.runOnce(superstructure::intake));
      //operatorXbox.b().onTrue(Commands.runOnce(superstructure::stow));

      /* Set Coral Scoring Location */
      buttonBox1.button(1).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 12)));
      buttonBox1.button(2).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 1)));
      buttonBox1.button(3).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 2)));
      buttonBox1.button(4).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Loction", 3)));
      buttonBox1.button(5).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 4)));
      buttonBox1.button(6).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 5)));
      buttonBox1.button(7).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 6)));
      buttonBox1.button(8).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 7)));
      buttonBox1.button(9).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 8)));
      buttonBox1.button(10).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 9)));
      buttonBox1.button(11).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 10)));
      buttonBox1.button(12).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 11)));
      
      /* White Buttons **OPERATOR FUNCTIONS** */
      buttonBox2.button(1).onTrue(new InstantCommand( () -> superstructure.setCoralLevel(1.0)));
      buttonBox2.button(2).onTrue(new InstantCommand( () -> superstructure.setCoralLevel(2.0)));
      buttonBox2.button(3).onTrue(new InstantCommand( () -> superstructure.setCoralLevel(3.0)));
      buttonBox2.button(4).onTrue(new InstantCommand( () -> superstructure.setCoralLevel(4.0)));
      buttonBox2.button(9).onTrue(new InstantCommand( () -> superstructure.clearAlgae(2.)));
      buttonBox2.button(10).onTrue(new InstantCommand( () -> superstructure.clearAlgae(3.)));
      buttonBox2.button(7).onTrue(new InstantCommand( () -> superstructure.intake()));
      buttonBox2.button(8).onTrue(new InstantCommand( () -> superstructure.goHome()));
      buttonBox2.button(6).whileTrue(new InstantCommand( () -> superstructure.spit()).repeatedly());
      buttonBox2.button(5).onTrue(new InstantCommand(() -> superstructure.startLifting()));
      buttonBox2.button(11).onTrue(new InstantCommand( () -> superstructure.enableManualOverride()));
      buttonBox2.button(11).onFalse(new InstantCommand( () -> superstructure.disableManualOverride()));
		  //buttonBox2.button(11).onTrue(new InstantCommand( () -> superstructure.moveCoralIn()));
		  //buttonBox2.button(12).onTrue(new InstantCommand( () -> superstructure.moveCoralOut()));

      /* Bobby's little button board */
      /* buttonBox.button(1).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 7)));
      buttonBox.button(2).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 8)));
      buttonBox.button(3).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 2)));
      buttonBox.button(4).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 1)));
      buttonBox.button(5).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 11)));
      buttonBox.button(6).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 12)));
      buttonBox.leftTrigger(.5).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 3)));
      buttonBox.rightTrigger(.5).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 4)));
      buttonBox.button(9).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 6)));
      buttonBox.button(10).onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 5)));
      buttonBox.povRight().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 9)));
      buttonBox.povLeft().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", 10)));*/
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    //drivebase.resetOdometry(pathPlannerAuto.getStartingPose());
    return autoChooser.getSelected();
  }

  /*
  private Command selectCommand() {
    double select = SmartDashboard.getNumber("Select Scoring Location", 0);
    int commandIndex = (int) Math.floor(select);
    SmartDashboard.putNumber("Debug-select command", commandIndex);
    switch (commandIndex) {
        case 1: return autoScoreSequenceCommand(1);
        case 2: return autoScoreSequenceCommand(2);
        case 3: return autoScoreSequenceCommand(3);
        case 4: return autoScoreSequenceCommand(4);
        case 5: return autoScoreSequenceCommand(5);
        case 6: return autoScoreSequenceCommand(6);
        case 7: return autoScoreSequenceCommand(7);
        case 8: return autoScoreSequenceCommand(8);
        case 9: return autoScoreSequenceCommand(9);
        case 10: return autoScoreSequenceCommand(10);
        case 11: return autoScoreSequenceCommand(11);
        case 12: return autoScoreSequenceCommand(12);
        default: return autoScoreSequenceCommand(4); // Default command
    }
  }*/
 /* public Command getAutoScoreCommand()
  {
    // An example command will be run in autonomous
    return new AutoScoreCommand(superstructure, drivebase);
  }*/

  public SuperstructureSubsystem getSuperstructure()
  {
    return superstructure;
  }

  public SwerveSubsystem getSwerveSubsystem()
  {
    return drivebase;
  }


  public Command visionIntake(){
    if (!Robot.isSimulation()) return drivebase.visionIntake();
    else return drivebase.visionIntake();
  }

  /**
   * 
   * @param withAutoRelease whether or not the robot should autorelease once it's at position
   * @return
   */
  public Command getScoreSequenceCommand(boolean withAutoRelease){
    double selectPose = SmartDashboard.getNumber("Select Scoring Location",0);
    Pose2d prescoreDrivePose = drivebase.getPrescorePose(selectPose);
    Pose2d scoreDrivePose = drivebase.getScorePose(selectPose);
    /*Command selectReefPoses = new InstantCommand(() -> {selectPose = SmartDashboard.getNumber("Select Scoring Location",0);
                                                        prescoreDrivePose = drivebase.getPrescorePose(selectPose);
                                                        scoreDrivePose = drivebase.getScorePose(selectPose);});*/
    Command driveToPrescore = drivebase.driveToPose(prescoreDrivePose);
    //Command driveToPrescore = drivebase.driveToTargetPosePID(prescoreDrivePose);
    Command driveToScore = drivebase.driveToTargetPosePID(scoreDrivePose);
    Command superStructureScore = new InstantCommand(() -> superstructure.startLifting());
    Command release = new InstantCommand(() -> superstructure.startReleasingCoral(true));
    //return  (new SequentialCommandGroup(selectReefPoses,driveToPrescore,driveToScore));*/
    //Command driveToPrescore = drivebase.driveToTargetPosePID(drivebase.getPrescorePose(SmartDashboard.getNumber("Select Scoring Location",0)));
    //Command driveToScore = drivebase.driveToTargetPosePID(drivebase.getScorePose(SmartDashboard.getNumber("Select Scoring Location",0)));
    Command autoScoreSequence = Commands.none();
    if(!withAutoRelease){
      autoScoreSequence = new SequentialCommandGroup(driveToPrescore, superStructureScore, driveToScore);}
    else{
      autoScoreSequence = new SequentialCommandGroup(driveToPrescore, superStructureScore, driveToScore, release);}
    
    return autoScoreSequence;
  }

  /**
   * Gets a score sequence command from user selected node
   * @param selection
   * @return
   */
  /*public Command autoScoreSequenceCommand(double selection){
    Command driveToPrescore = drivebase.driveToTargetPosePID(drivebase.getPrescorePose(selection));
    Command driveToScore = drivebase.driveToTargetPosePID(drivebase.getScorePose(selection));
    Command autoScoreSequence = new SequentialCommandGroup(driveToPrescore, driveToScore);
    
    return autoScoreSequence;
  }
*/
  public void setDriveMode()
  {
    configureBindings();
  }

  public void elevatorHoldPos(){
   superstructure.elevatorHoldPos();
    
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  /*private Command pidToPose(Pose2d targetPosePID){
    Pose2d currentPose = drivebase.getPose();
    TrapezoidProfile.Constraints xyConstraints = new Constraints(3 ,3);
    TrapezoidProfile.Constraints thetaConstraints = new Constraints(Math.PI*2,Math.PI*4);
    
    ProfiledPIDController xcontroller = new ProfiledPIDController(.2, 0, 0, xyConstraints);
    ProfiledPIDController ycontroller = new ProfiledPIDController(.2, 0, 0, xyConstraints);
    ProfiledPIDController thetacontroller = new ProfiledPIDController(.1, 0, 0, thetaConstraints);
    
    //PIDController xcontroller = new PIDController(1, 0, 0);
    //PIDController ycontroller = new PIDController(1, 0, 0);
    //PIDController thetacontroller = new PIDController(1, 0, 0);
    //thetacontroller.enableContinuousInput(-Math.PI,Math.PI);
    
    count++;
    SmartDashboard.putNumber("RobotContainer PID Debug", count);
    SwerveInputStream pidDrive = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> xcontroller.calculate(currentPose.getX(), targetPosePID.getX()) * -1,
        () -> ycontroller.calculate(currentPose.getY(), targetPosePID.getY()) * -1)//
      //.withControllerRotationAxis(driverXbox::getRightX)
      .withControllerRotationAxis(() -> thetacontroller.calculate(currentPose.getRotation().getRadians(), targetPosePID.getRotation().getRadians()) * -1) //
      .deadband(0.001)
      .scaleTranslation(.8)
      .allianceRelativeControl(false);

    Command drivePIDtoPose = drivebase.driveFieldOriented(pidDrive);
    return drivePIDtoPose;
  } */
  
}
