package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class L2Command extends Command {
    SuperstructureSubsystem structure;

    public L2Command(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        if(structure.hasCoral = true){
         
        structure.setCoralLevel(2.0);
        structure.startLifting();

        }
    }
    @Override
    public void execute() {
        if(structure.atPositionScoring() == true /*&& structure.sequenceState == 1*/){
            // if(getFPGATimestamp)
            structure.startReleasingCoral(false);
        }
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return structure.scoringCoral == false;
    }
}
