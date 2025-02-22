package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class L1Command extends Command {
    SuperstructureSubsystem structure;

    public L1Command(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        if(structure.hasCoral = true){
        structure.startLifting();
        structure.liftL1();

        if(structure.atPosition() == true && structure.sequenceState == 1){
            structure.releaseCoral();
            }
        }
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return structure.scoringCoral = false;
    }
}
