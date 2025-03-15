package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;

public class L4Command extends Command {
    SuperstructureSubsystem structure;

    public L4Command(SuperstructureSubsystem structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        if(structure.hasCoral = true){ 
            structure.setCoralLevel(4.0);
            structure.startLifting();
            }
        }
    
    @Override
    public void execute() {
        if(structure.atPosition() == true && structure.sequenceState == 1){
            structure.startReleasingCoral(false);
        }
        
    }

    @Override
    public void end(boolean interrupted){
        //structure.goHome();
    }
    
    public boolean isFinished() {
        return structure.scoringCoral == true;
    }
}
