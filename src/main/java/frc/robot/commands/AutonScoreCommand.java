// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.superstructure.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import drivebase.driveToPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Autoscore command 
 * @param m_RobotContainer
 * @param m_SuperstructureSub
 * @param m_DrivetrainSub
 * @param ScoringLocation
 * @param ScoringLevel
 * send it the level and position for scoring, and it'll do the rest!
 */
public class AutonScoreCommand extends Command {
  SuperstructureSubsystem m_SuperstructureSub;
  SwerveSubsystem m_DrivetrainSub;
  RobotContainer m_RobotContainer;
  Double ScoringLocation;
  Double ScoringLevel;
  Double timerStart;
/**
 * creates autoscore command
 * @param m_SuperstructureSub
 * @param m_DrivetrainSub
 * @param ScoringLocation
 * @param ScoringLevel
 */
public AutonScoreCommand(RobotContainer m_RobotContainer, SuperstructureSubsystem m_SuperstructureSub, SwerveSubsystem m_DrivetrainSub, double ScoringLocation, double ScoringLevel) {
    this.m_RobotContainer = m_RobotContainer;
    this.m_SuperstructureSub = m_SuperstructureSub;
    this.m_DrivetrainSub = m_DrivetrainSub;
  //public AutonScoreCommand(double ScoringLocation, double ScoringLevel) {
    this.ScoringLocation = ScoringLocation;
    this.ScoringLevel = ScoringLevel; 

    // Use addRequirements() here to declare subsystem dependencies.

    //addRequirements(m_SuperstructureSub);
    //addRequirements(m_DrivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timerStart = Timer.getFPGATimestamp();

    Command autoScore = m_RobotContainer.getScoreSequenceCommand(true);
    CommandScheduler.getInstance().schedule(autoScore);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancelAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
