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
    public SuperstructureState PassOff;
    public SuperstructureState StowClear;
    public SuperstructureState StowClearIntakeDeployed;
    public SuperstructureState StowCoral;
    public SuperstructureState StowAlgae;
    public SuperstructureState PreScoreCoral;

    public SuperstructureStates(){
        StartingConfig = new SuperstructureState(0, 0, 0);
        Home = new SuperstructureState(0, 0, 0);
        Intake = new SuperstructureState(0, 0, 0);
        CoralL1 = new SuperstructureState(0, 0, 0);
        CoralL2 = new SuperstructureState(0, 0, 0);
        CoralL3 = new SuperstructureState(0, 0, 0);
        CoralL4 = new SuperstructureState(0, 0, 0);

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(0, 0, 0);
        CoralPreL2 = new SuperstructureState(0, 0, 0);
        CoralPreL3 = new SuperstructureState(0, 0, 0);
        CoralPreL4 = new SuperstructureState(0, 0, 0);

        CoralPostL1 = new SuperstructureState(0, 0, 0);
        CoralPostL2 = new SuperstructureState(0, 0, 0);
        CoralPostL3 = new SuperstructureState(0, 0, 0);
        CoralPostL4 = new SuperstructureState(0, 0, 0);

        PassOff = new SuperstructureState(0, 0, 0);
        StowClear = new SuperstructureState(0, 0, 0); //intended as a safe position with endeffector clear from crashing with elevator stages
        StowClearIntakeDeployed = new SuperstructureState(0, 0, 0);
        StowCoral = new SuperstructureState(0, 0, 0);
        StowAlgae = new SuperstructureState(0, 0, 0);
        PreScoreCoral = new SuperstructureState(0, 0, 0);
    }

}
