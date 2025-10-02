package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureStates {

    public SuperstructureState StartingConfig;
    public SuperstructureState Home;
    public SuperstructureState Intake;
    public SuperstructureState IntakeWobble;

    public SuperstructureState grabAlgaeL2;
    public SuperstructureState grabAlgaeL3;

    public SuperstructureState CoralL1Manual;
    public SuperstructureState CoralL2Manual;
    public SuperstructureState CoralL3Manual;
    public SuperstructureState CoralL4Manual;

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
    public SuperstructureState climb1;
    public SuperstructureState climb2;
    public SuperstructureState climb3;
    public SuperstructureState L1Intake;
    

    public SuperstructureStates(){
        StartingConfig = new SuperstructureState(0, 0.0, 0.0);
        Home = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 1.5 +.2 ); //was 1.5
        Intake = new SuperstructureState(-14*Constants.endEffectorPivotGearRatio / 360, 0.2, 10.8); //intake 97, eepiv -23 //was -23
        IntakeWobble = new SuperstructureState((-12)*Constants.endEffectorPivotGearRatio / 360, 0.2, 10.2 ); //intake 97, eepiv -23 //was -18

        L1Intake = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2, 5.8);

        //grabAlgaeL2 = new SuperstructureState(80*Constants.endEffectorPivotGearRatio / 360, 14 / 2.25, 15/9);
        //grabAlgaeL3 = new SuperstructureState(80*Constants.endEffectorPivotGearRatio / 360, 33 / 2.25, 15/9);

        grabAlgaeL2 = new SuperstructureState(60*Constants.endEffectorPivotGearRatio / 360, 24 / 2.25, 15/9); //CONTRA - DAY 1
        grabAlgaeL3 = new SuperstructureState(60*Constants.endEffectorPivotGearRatio / 360, 43 / 2.25, 15/9); //CONTRA - DAY 1


        //grabAlgaeL2 = new SuperstructureState(90*Constants.endEffectorPivotGearRatio / 360, 24 / 2.25, 15/9);
        //grabAlgaeL3 = new SuperstructureState(90*Constants.endEffectorPivotGearRatio / 360, 43 / 2.25, 15/9);
        /**
         * use these for manual scoring (i.e. )
         */
        CoralL1Manual = new SuperstructureState(80*Constants.endEffectorPivotGearRatio / 360, 5/ 2.25, 15 /9);
        CoralL2Manual = new SuperstructureState(85*Constants.endEffectorPivotGearRatio / 360, 16/ 2.25, 15/9);
        CoralL3Manual = new SuperstructureState(85*Constants.endEffectorPivotGearRatio / 360, 38/ 2.25, 15/9);
        CoralL4Manual = new SuperstructureState(70*Constants.endEffectorPivotGearRatio / 360, 72/ 2.25, 15/9); // LAR L4 manual position

        CoralL1 = new SuperstructureState(30*Constants.endEffectorPivotGearRatio / 360, 12/ 2.25, 15/9);
        CoralL2 = new SuperstructureState(110*Constants.endEffectorPivotGearRatio / 360, 3.69, 15/9);
        CoralL3 = new SuperstructureState(110*Constants.endEffectorPivotGearRatio / 360, 12.33, 15/9);
        CoralL4 = new SuperstructureState(6.34, 72/ 2.25, 15/9); // Before Contra playoffs

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 8/ 2.25,  15/9);
        CoralPreL2 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 28/ 2.25, 15/9);
        CoralPreL3 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 50/ 2.25, 15/9);
        CoralPreL4 = new SuperstructureState(20*Constants.endEffectorPivotGearRatio / 360, 33.7, 15/9);

//        CoralPostL1 = new SuperstructureState(35*Constants.endEffectorPivotGearRatio / 360, 8/ 2.25,  15/9);
//        CoralPostL2 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 31.86/ 2.25, 15/9);
//        CoralPostL3 = new SuperstructureState(11*Constants.endEffectorPivotGearRatio / 360, 48/ 2.25, 15/9);
//        CoralPostL4 = new SuperstructureState(20*Constants.endEffectorPivotGearRatio / 360, 33.7, 15/9);

        AlgaePassOff = new SuperstructureState(-3*Constants.endEffectorPivotGearRatio / 360, 4/ 2.25, 4/9); //
        StowEEClear  = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2/ 2.25, 15/9); //intended as a safe position with endeffector clear from crashing with elevator stages or the intake
        StowClearIntakeDeployed = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 1/ 2.25, 93.4/9);
        StowWithCoral = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2/ 2.25, 15/9);
        StowWithAlgae = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 0.2/ 2.25, 15/9);
        PreScoreCoral = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 10/ 2.25, 15/9);

        climb1 = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 1/ 2.25, 92/9);
       
        climb2 = new SuperstructureState(25*Constants.endEffectorPivotGearRatio / 360, 1/ 2.25, 60/9);
        climb3 = new SuperstructureState(90*Constants.endEffectorPivotGearRatio / 360, 0/ 2.25, 20/9);
       
    }

}
