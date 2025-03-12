// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutonScoreCommand;
import frc.robot.commands.VisionIntakeCommand;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private double remapLimiterCount = 0;
  public SuperstructureSubsystem m_SuperstructureSubsystem = new SuperstructureSubsystem();
  
  private Timer disabledTimer;
  final         CommandXboxController driverXbox = new CommandXboxController(0);

  // final LEDs m_LEDs = new LEDs();

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  /**
   * This function is run when the robot is first started up and should be used for any initialization code.
   */
  @Override
  public void robotInit()
  {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    m_robotContainer.setDriveMode();//added 2-28-25
    
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // Create a timer to disable motor brake a few seconds after disable.  This will let the robot stop
    // immediately when disabled, but then also let it be pushed more 
    disabledTimer = new Timer();
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics that you want ran
   * during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic()
  {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    //

    //driverXbox.leftBumper().whileTrue(m_robotContainer.autoscoreDriveCommand());
    
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_robotContainer.setDriveMode();
    disabledTimer.reset();
    disabledTimer.start();
    // m_LEDs.setLightMode(0);
  }

  @Override
  public void disabledPeriodic()
  {
    m_SuperstructureSubsystem.lightMode = 0;

    //m_LEDs.setLightMode(0);
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.schedule();
    }
    new SequentialCommandGroup(
      new AutonScoreCommand(m_robotContainer, m_robotContainer.getSuperstructure(), m_robotContainer.getSwerveSubsystem(),8.0, 4.0).withTimeout(.02),
      new AutonScoreCommand(m_robotContainer, m_robotContainer.getSuperstructure(), m_robotContainer.getSwerveSubsystem(),8.0, 4.0),
      new VisionIntakeCommand(m_robotContainer, m_robotContainer.getSuperstructure(), m_robotContainer.getSwerveSubsystem(),5.0),
      new AutonScoreCommand(m_robotContainer, m_robotContainer.getSuperstructure(), m_robotContainer.getSwerveSubsystem(),6.0, 4.0)
      ).schedule(); 
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
     m_robotContainer.setDriveMode();

     //driverXbox.rightBumper().whileTrue(m_robotContainer.getScoreSequenceCommand());
    // m_robotContainer.setMotorBrake(true);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic()
  {
    
  }

  @Override
  public void testInit()
  {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setDriveMode();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic()
  {
  }

  /**
   * This function is called once when the robot is first started up.
   */
  @Override
  public void simulationInit()
  {
  }

  /**
   * This function is called periodically whilst in simulation.
   */
  @Override
  public void simulationPeriodic()
  {
  }
}