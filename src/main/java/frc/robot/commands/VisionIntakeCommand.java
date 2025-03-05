package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
//import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class VisionIntakeCommand extends Command {
    RobotContainer robotContainer;
    SuperstructureSubsystem structure;
    double timerStart = 0;
    double timeLimit = 3;

    public VisionIntakeCommand(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        structure.goHome();
        structure.intake();
        timerStart = Timer.getFPGATimestamp();
    }
    @Override
    public void execute() {
        robotContainer.visionIntake();
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return (structure.hasCoral == true || Timer.getFPGATimestamp() - timerStart > timeLimit);
    }
}
