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

<<<<<<< HEAD
            if(structure.atPosition() == true && structure.sequenceState == 1){
                structure.releaseCoral();
                }
=======
        if(structure.atPosition() == true && structure.sequenceState == 1){
            structure.releaseCoral();
            }
>>>>>>> 57e3d9c0d80e455cc9904ca42513af889e61eba2
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
