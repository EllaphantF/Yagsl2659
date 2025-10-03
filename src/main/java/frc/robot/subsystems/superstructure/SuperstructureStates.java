package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureStates {

    public SuperstructureState StartingConfig;
    public SuperstructureState Home;
    public SuperstructureState Intake;

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
    public SuperstructureState bargeAlgae;
    public SuperstructureState processorAlgae;

    

    public SuperstructureStates(){
        StartingConfig = new SuperstructureState(0., 0.2, 0., 0.);
        Home = new SuperstructureState(0., 0.2, 0., 4. );
        Intake = new SuperstructureState(0., 0.2, 0.,0.); //intake 97, eepiv -23

        grabAlgaeL2 = new SuperstructureState(0., 14., 15., 0.);

        grabAlgaeL3 = new SuperstructureState(0., 33., 15., 0.);
        /**
         * use these for manual scoring (i.e. )
         */
        CoralL1Manual = new SuperstructureState(0., 5., 15., 0.);
        CoralL2Manual = new SuperstructureState(0., 16., 15., 0.);
        CoralL3Manual = new SuperstructureState(0., 38., 15., 0.);
        CoralL4Manual = new SuperstructureState(0., 76., 15., 0.);

        CoralL1 = new SuperstructureState(0., 12., 15., 0.);
        CoralL2 = new SuperstructureState(0., 11., 15., 0.);
        CoralL3 = new SuperstructureState(0., 30., 15., 0.);
        CoralL4 = new SuperstructureState(0., 72., 15., 0.); // elevator at 71

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        CoralPreL1 = new SuperstructureState(0., 8.,  15., 0.);
        CoralPreL2 = new SuperstructureState(0., 26., 15., 0.);
        CoralPreL3 = new SuperstructureState(0., 48., 15., 0.);
        CoralPreL4 = new SuperstructureState(0., 76.5, 15., 0.);

        CoralPostL1 = new SuperstructureState(0., 8.,  15., 0.);
        CoralPostL2 = new SuperstructureState(0., 31.86, 15., 0.);
        CoralPostL3 = new SuperstructureState(0., 48., 15., 0.);
        CoralPostL4 = new SuperstructureState(0., 76.5, 15., 0.);

        AlgaePassOff = new SuperstructureState(0., 4., 4., 0.); //
        StowEEClear  = new SuperstructureState(0., 0.2, 15., 0.); //intended as a safe position with endeffector clear from crashing with elevator stages or the intake
        StowClearIntakeDeployed = new SuperstructureState(0., 1., 90., 0.);
        StowWithCoral = new SuperstructureState(0., 0.2, 15., 0.);
        StowWithAlgae = new SuperstructureState(0., 0.2, 15., 0.);
        PreScoreCoral = new SuperstructureState(0., 10., 15., 0.);

        bargeAlgae = new SuperstructureState(0.,0.,0.,0.);
        processorAlgae = new SuperstructureState(0.,0.,0.,0.);

        climb1 = new SuperstructureState(0., 1., 92., 0.);
        climb2 = new SuperstructureState(0., 1., 60., 0.);
        climb3 = new SuperstructureState(0., 0., 20., 0.);
       
    }

}
