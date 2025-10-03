package frc.robot.subsystems.superstructure;

public class SuperstructureState {
    public double arm;
    public double elevator;
    public double climb;
    public double ee;

        /**
         * 
         * @param climb - endeffector arm angle - degrees from starting position
         * @param elevator - elevator height - motor turns
         * @param arm - intake angle - degrees from starting position
         * @param ee - endeffector wrist angle - degrees from starting position 
         */
        public SuperstructureState(double climb, double elevator, double arm, double ee){
            this.climb = climb;
            this.elevator = elevator;
            this.arm = arm;
            this.ee = ee;
        }
    
}