package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

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
        StartingConfig = new SuperstructureState(0, 0.2, 0);
        Home = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 15);
        Intake = new SuperstructureState(-22*Constants.endEffectorPivotGearRatio / 360, 0.2, 90);
        CoralL1 = new SuperstructureState(80*Constants.endEffectorPivotGearRatio / 360, 3, 15);
        CoralL2 = new SuperstructureState(85*Constants.endEffectorPivotGearRatio / 360, 16, 15);
        CoralL3 = new SuperstructureState(80*Constants.endEffectorPivotGearRatio / 360, 36.5, 15);
        CoralL4 = new SuperstructureState(50*Constants.endEffectorPivotGearRatio / 360, 76, 15);

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 8,  15);
        CoralPreL2 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 26, 15);
        CoralPreL3 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 48, 15);
        CoralPreL4 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 76, 15);

        CoralPostL1 = new SuperstructureState(35*Constants.endEffectorPivotGearRatio / 360, 8,  15);
        CoralPostL2 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 31.86, 15);
        CoralPostL3 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 48, 15);
        CoralPostL4 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 76, 15);

        AlgaePassOff = new SuperstructureState(-3*Constants.endEffectorPivotGearRatio / 360, 4, 4); //
        StowEEClear  = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 15); //intended as a safe position with endeffector clear from crashing with elevator stages or the intake
        StowClearIntakeDeployed = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 1, 92);
        StowWithCoral = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 15);
        StowWithAlgae = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 15);
        PreScoreCoral = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 10, 15);
    }

}
