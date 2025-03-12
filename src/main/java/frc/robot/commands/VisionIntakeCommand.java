package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


/**
 * Autoscore command 
 * @param m_RobotContainer
 * @param m_SuperstructureSub
 * @param m_DrivetrainSub
 * @param timeLimit
 * send it the level and position for scoring, and it'll do the rest!
 */
public class VisionIntakeCommand extends Command {
    SuperstructureSubsystem m_SuperstructureSub;
    SwerveSubsystem m_DrivetrainSub;
    RobotContainer m_RobotContainer;
    Double ScoringLocation;
    Double ScoringLevel;
    Double timerStart;
    Double timeLimit;

    public VisionIntakeCommand(RobotContainer m_RobotContainer, SuperstructureSubsystem structure, SwerveSubsystem m_DrivetrainSub, Double timeLimit) {
        this.m_SuperstructureSub = structure;
        this.m_DrivetrainSub = m_DrivetrainSub;
        this.m_RobotContainer = m_RobotContainer;
        this.timeLimit = timeLimit;
    }

    @Override
    public void initialize() {
        m_SuperstructureSub.goHome();
        m_SuperstructureSub.intake();
        timerStart = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
        m_RobotContainer.visionIntake();
    }

    @Override
    public void end(boolean interrupted){
        m_SuperstructureSub.goHome();
    }
    
    public boolean isFinished() {
        return (m_SuperstructureSub.hasCoral == true || Timer.getFPGATimestamp() - timerStart > timeLimit);
    }
}
