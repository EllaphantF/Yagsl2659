package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class L3L4PosCommand extends Command {
    SuperstructureSubsystem structure;

    public L3L4PosCommand(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        structure.TARGETSTATE = structure.STATE.StowPreL34;
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        
    }
    
    public boolean isFinished() {
        return true;
    }
}
