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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class AutoScoreCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  SuperstructureSubsystem m_SuperstructureSub;
  SwerveSubsystem m_DrivetrainSub;
  Double ScoringLocation;
  Double ScoringLevel;
  Integer State = 0;
  Boolean AutoRelease;
  Pose2d scoreDrivePose;
  Pose2d prescoreDrivePose;
  Command driveToPrescorePose;
  Command driveToScorePose;
  Integer count = 0;
/**
 * creates autoscore command
 * @param m_SuperstructureSub
 * @param m_DrivetrainSub
 * //looks up level and location from smartdash for scoring position and superstructure settings, looks up boolean for autorelease on smartdash for if we're wanting the robot to auto-drop the gamepiece. If we autorelease, the robot will also auto retract and drive back 
 */
  public AutoScoreCommand(SuperstructureSubsystem m_SuperstructureSub, SwerveSubsystem m_DrivetrainSub) {
    this.m_SuperstructureSub = m_SuperstructureSub;
    this.m_DrivetrainSub = m_DrivetrainSub;
    //this.AutoRelease = AutoRelease;
    SmartDashboard.putNumber("Select Scoring Level",SmartDashboard.getNumber("Select Scoring Level",5));
    SmartDashboard.putBoolean("Scoring Autorelease",false);
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.m_DrivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ScoringLocation = SmartDashboard.getNumber("Select Scoring Location",5);
    ScoringLevel = SmartDashboard.getNumber("Select Scoring Level",4);
    AutoRelease = SmartDashboard.getBoolean("Scoring Autorelease", false);
    SmartDashboard.putNumber("Autocommand Count",count);
    count = count + 1;

    Command superstructurePrescore = new InstantCommand(()-> m_SuperstructureSub.setPreScoreCoralPos(ScoringLevel));

    prescoreDrivePose = m_DrivetrainSub.getPrescorePose(ScoringLocation);
    scoreDrivePose = m_DrivetrainSub.getScorePose(ScoringLocation);

    driveToPrescorePose = m_DrivetrainSub.driveToPose(prescoreDrivePose);

    driveToScorePose = m_DrivetrainSub.driveToTargetPosePID(scoreDrivePose);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Autocommand Count",count);
    //count = count +1;
    switch (State) {
      case 0:
        m_DrivetrainSub.driveToPose(prescoreDrivePose);
        //CommandScheduler.getInstance().schedule(driveToPrescorePose);
        SmartDashboard.putNumber("Autocommand Count",count);
        count = count + 1;
        if(driveToPrescorePose.isFinished()) State++;
        break;
      case 1:
        driveToScorePose.execute();        
        break;
      default:
        break;
    }
//    m_DrivetrainSub.driveToPose(prescoreDrivePose);

    //m_DrivetrainSub.driveToTargetPosePID(scoreDrivePose);

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
