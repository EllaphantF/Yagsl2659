package frc.robot.subsystems.superstructure;

public class SuperstructureStates {

    public SuperstructureState StartingConfig;
    public SuperstructureState Home;
    public SuperstructureState Intake;
    public SuperstructureState CoralL1;
    public SuperstructureState CoralL2;
    public SuperstructureState CoralL3;
    public SuperstructureState CoralL4;
    public SuperstructureState CoralPreL1;
    public SuperstructureState CoralPreL2;
    public SuperstructureState CoralPreL3;
    public SuperstructureState CoralPreL4;
    public SuperstructureState CoralPostL1;
    public SuperstructureState CoralPostL2;
    public SuperstructureState CoralPostL3;
    public SuperstructureState CoralPostL4;
    public SuperstructureState AlgaePassOff;
    public SuperstructureState StowEEClear;
    public SuperstructureState StowClearIntakeDeployed;
    public SuperstructureState StowWithCoral;
    public SuperstructureState StowWithAlgae;
    public SuperstructureState PreScoreCoral;

    public SuperstructureStates(){
        StartingConfig = new SuperstructureState(0, 0, 0);
        Home = new SuperstructureState(0, 0, 15);
        Intake = new SuperstructureState(-10, 0.2, 45);
        CoralL1 = new SuperstructureState(95, 3, 15);
        CoralL2 = new SuperstructureState(85, 10, 15);
        CoralL3 = new SuperstructureState(85, 40, 15);
        CoralL4 = new SuperstructureState(75, 72, 15);

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(15, 3,  15);
        CoralPreL2 = new SuperstructureState(15, 11, 15);
        CoralPreL3 = new SuperstructureState(15, 41, 15);
        CoralPreL4 = new SuperstructureState(15, 73, 15);

        CoralPostL1 = new SuperstructureState(15, 3,  15);
        CoralPostL2 = new SuperstructureState(15, 11, 15);
        CoralPostL3 = new SuperstructureState(15, 41, 15);
        CoralPostL4 = new SuperstructureState(15, 73, 15);

        AlgaePassOff = new SuperstructureState(-3, 4, 4); //
        StowEEClear  = new SuperstructureState(15, 0, 15); //intended as a safe position with endeffector clear from crashing with elevator stages or the intake
        StowClearIntakeDeployed = new SuperstructureState(15, 1, 45);
        StowWithCoral = new SuperstructureState(5, 0, 15);
        StowWithAlgae = new SuperstructureState(15, 0, 15);
        PreScoreCoral = new SuperstructureState(15, 10, 15);
    }

}
