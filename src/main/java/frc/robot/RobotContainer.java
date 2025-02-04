// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import java.io.File;

import drivebase.driveToPose;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  private final SuperstructureSubsystem superstructure = new SuperstructureSubsystem();
      // the main mechanism object
  Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
  MechanismRoot2d root = mech.getRoot("intake", 2, 0);
  
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

  Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);

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

  Command driveSetpointGenSim = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleSim);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
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
                                driveFieldOrientedDirectAngle :
                                driveFieldOrientedAnglularVelocity);

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
    }
    if (DriverStation.isTest()) //BVN 1-26-25 note, if we enable test itll bind these, but it doesnt unbind them if we enable tele, it just over writes
                                // so some of the button bindings are still there from enabling one mode(i.e. test), then enabling the other (i.e. tele).
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
      driverXbox.b().whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      /*driverXbox.leftBumper().whileTrue(drivebase.driveToPose(
        new Pose2d(new Translation2d(3.5 ,3.5),Rotation2d.fromDegrees(30))));
      driverXbox.rightBumper().whileTrue(drivebase.driveToPose(
        new Pose2d(new Translation2d(3,3),Rotation2d.fromDegrees(0))));*/
    } else
    {
      driverXbox.povLeft().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", SmartDashboard.getNumber("Select Scoring Location",0)-.5)));
      driverXbox.povRight().onTrue(new InstantCommand( () -> SmartDashboard.putNumber("Select Scoring Location", SmartDashboard.getNumber("Select Scoring Location",0)+.5)));

      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      // driverXbox.y().whileTrue(drivebase.aimAtSpeaker(2));
      //driverXbox.start().whileTrue(Commands.none());
      driverXbox.start().whileTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(new Translation2d(5,5),new Rotation2d(0)))));
      driverXbox.back().whileTrue(Commands.none());
      //driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      //driverXbox.rightBumper().onTrue(Commands.none());
      //driverXbox.rightBumper().onTrue(new InstantCommand(() -> getAutoDriveCommand()));
      //driverXbox.rightBumper().whileTrue(new RunCommand(() -> getAutoDriveCommand()));
      
//      driverXbox.rightBumper().whileTrue(getAutoDriveCommand()); //works but only goes to the first pose - i.e. cant update target pose
//      driverXbox.rightBumper().whileTrue(autoScoreSequenceCommand()); //running this one in robot periodic constantly updates the path to the pose 

      //driverXbox.rightBumper().whileTrue(new InstantCommand(() -> drivebase.autoDriveToReef()));
      
      operatorXbox.a().onTrue(Commands.runOnce(superstructure::intake));
      operatorXbox.b().onTrue(Commands.runOnce(superstructure::stow));
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
    return drivebase.getAutonomousCommand("New Auto");
  }

  public Command autoScoreSequenceCommand(){
    double selectPose = SmartDashboard.getNumber("Select Scoring Location",0);
    Pose2d prescoreDrivePose = drivebase.getPrescorePose(selectPose);
    Pose2d scoreDrivePose = drivebase.getScorePose(selectPose);
  return  (new SequentialCommandGroup(drivebase.driveToPose(prescoreDrivePose),drivebase.driveToTargetPosePID(scoreDrivePose)));
  }

  public void setDriveMode()
  {
    configureBindings();
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
