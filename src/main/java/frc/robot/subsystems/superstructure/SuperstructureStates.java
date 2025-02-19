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
        Home = new SuperstructureState(2, 0, 6);
        Intake = new SuperstructureState(-1, 0.2, 25);
        CoralL1 = new SuperstructureState(12, 3, 6);
        CoralL2 = new SuperstructureState(10, 10, 6);
        CoralL3 = new SuperstructureState(10, 40, 6);
        CoralL4 = new SuperstructureState(8, 72, 6);

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(6, 3, 6);
        CoralPreL2 = new SuperstructureState(6, 11, 6);
        CoralPreL3 = new SuperstructureState(6, 41, 6);
        CoralPreL4 = new SuperstructureState(6, 73, 6);

        CoralPostL1 = new SuperstructureState(6, 3,6);
        CoralPostL2 = new SuperstructureState(6, 11, 6);
        CoralPostL3 = new SuperstructureState(6, 41, 6);
        CoralPostL4 = new SuperstructureState(6, 73,6);

        AlgaePassOff = new SuperstructureState(-3, 4, 4); //
        StowEEClear = new SuperstructureState(5, 1, 5); //intended as a safe position with endeffector clear from crashing with elevator stages or the intake
        StowClearIntakeDeployed = new SuperstructureState(5, 1, 25);
        StowWithCoral = new SuperstructureState(0, 0, 0);
        StowWithAlgae = new SuperstructureState(0, 0, 0);
        PreScoreCoral = new SuperstructureState(0, 0, 0);
    }

}
