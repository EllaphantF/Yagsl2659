package frc.robot.subsystems.superstructure;

public class SuperstructureState {
    public double pivot;
    public double elevator;
    public double intake;
    
    /**
     * 
     * @param pivot - endeffector angle - degrees from starting position
     * @param elevator - elevator height - motor turns
     * @param intake - intake angle - degrees from starting position
     */
    public SuperstructureState(double pivot, double elevator, double intake){
        this.pivot = pivot;
        this.elevator = elevator;
        this.intake = intake;
    }
    
}