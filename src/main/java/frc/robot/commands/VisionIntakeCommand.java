package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionIntakeCommand extends Command {
    RobotContainer m_robotContainer;
    SuperstructureSubsystem m_structure;
    double timerStart = 0;
    double timeLimit = 3;

    public VisionIntakeCommand(RobotContainer m_RobotContainer, SuperstructureSubsystem m_SuperstructureSub, SwerveSubsystem m_DrivetrainSub) {
        this.m_structure = m_structure;
        this.m_robotContainer = m_robotContainer;

    }

    @Override
    public void initialize() {
        m_structure.goHome();
        m_structure.intake();
        timerStart = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
        m_robotContainer.visionIntake();
    }

    @Override
    public void end(boolean interrupted){
        m_structure.goHome();
    }
    
    public boolean isFinished() {
        return (m_structure.hasCoral == true || Timer.getFPGATimestamp() - timerStart > timeLimit);
    }
}
