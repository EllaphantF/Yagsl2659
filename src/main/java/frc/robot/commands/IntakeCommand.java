package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class IntakeCommand extends Command {
    SuperstructureSubsystem structure;

    public IntakeCommand(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        structure.goHome();
        structure.intake();
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return structure.intaking = false;
    }
}
