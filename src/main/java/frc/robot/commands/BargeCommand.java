package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class BargeCommand extends Command {
    SuperstructureSubsystem structure;

    public BargeCommand(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        structure.goToBargeAlgaeScoring();
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        
    }
    
    public boolean isFinished() {
        return structure.hasAlgae = false;
    }
}
