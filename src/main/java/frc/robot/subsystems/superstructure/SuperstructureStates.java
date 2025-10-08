package frc.robot.subsystems.superstructure;

import frc.robot.Constants;

public class SuperstructureStates {

    public SuperstructureState StartingConfig;
    public SuperstructureState Home;
    public SuperstructureState Intake;

    public SuperstructureState groundIntakeAlgae;
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
    public SuperstructureState StowWithAlgaeL2;
    public SuperstructureState StowWithAlgaeL3;
    public SuperstructureState PreScoreCoral;
    public SuperstructureState climb1;
    public SuperstructureState climb2;
    public SuperstructureState climb3;
    public SuperstructureState bargeAlgae;
    public SuperstructureState processorAlgae;

    

    public SuperstructureStates(){

        /*
         * 10-7-2025 ACE Updates!! 
         * It's SUPER important that we check each of the positions that we will use, so adding a checkbox system as comments
         * 
         *  checked:   /*(X)*/
         /* unchecked: /*( )*/
         /*
         *
         */

        /*(x )*/ StartingConfig = new SuperstructureState(0., 0.0, 0., 0.);
        /*(x )*/ Home = new SuperstructureState(0., 0.2, 3., 0. );
        /*(x )*/ Intake = new SuperstructureState(0., 2.8+.3, 0.,0.); //intake 97, eepiv -23

        /*(x )*/ groundIntakeAlgae = new SuperstructureState(0., 0.1, 5.8, 9.5);

        /*(x )*/ grabAlgaeL2 = new SuperstructureState(0., 0.1, 15.6, 11.55);

        /*(x )*/ grabAlgaeL3 = new SuperstructureState(0., 17.3, 15.6, 11.55);
                /**
                 * use these for manual scoring (i.e. )
                 */
        /*( )*/ CoralL1Manual = new SuperstructureState(0., 5., 15., 0.);
        /*( )*/ CoralL2Manual = new SuperstructureState(0., 16., 15., 0.);
        /*( )*/ CoralL3Manual = new SuperstructureState(0., 38., 15., 0.);
        /*( )*/ CoralL4Manual = new SuperstructureState(0., 76., 15., 0.);

        /*(x )*/ CoralL1 = new SuperstructureState(0., 0.1, 9., 6.7);
        /*(x )*/ CoralL2 = new SuperstructureState(0., 0.1, 14., 14.1);
        /*(x )*/ CoralL3 = new SuperstructureState(0., 17.3, 13.8, 14.1);
        /*(x )*/ CoralL4 = new SuperstructureState(0., 26.5, 19.8, 19.4); // elevator at 71

        /* need to send the elevator up with endeffector in so we dont crash into the reef */
        /*(x )*/ CoralPreL1 = new SuperstructureState(0., 0.1, 9., 6.7);
        /*(x )*/ CoralPreL2 = new SuperstructureState(0., 1.2, 17, 14.8);
        /*(x )*/ CoralPreL3 = new SuperstructureState(0., 17.3+1.5, 17, 14.8);
        /*(x )*/ CoralPreL4 = new SuperstructureState(0., 26.5, 22.0, 19.4);

        /*ACE - We never actually used them in season but might need them for Ace */
        CoralPostL1 = new SuperstructureState(0., 8.,  15., 0.);
        CoralPostL2 = new SuperstructureState(0., 31.86, 15., 0.);
        CoralPostL3 = new SuperstructureState(0., 48., 15., 0.);
        CoralPostL4 = new SuperstructureState(0., 76.5, 15., 0.);

//        AlgaePassOff = new SuperstructureState(0., 4., 4., 0.); // might not be used on Ace
        StowEEClear  = new SuperstructureState(0., 0.2, 1., 0.); // 
        StowClearIntakeDeployed = new SuperstructureState(0., 1., 1., 0.);
        StowWithCoral = new SuperstructureState(0., 0.2, 1., 0.);
        StowWithAlgae = new SuperstructureState(0., 0.2, 4., 0.2);
        StowWithAlgaeL2 = new SuperstructureState(0., 0.2, 19., 13.);
        StowWithAlgaeL3 = new SuperstructureState(0., 16.5, 19., 13.);
        PreScoreCoral = new SuperstructureState(0., .2, 1., 0.);

        /*(x )*/bargeAlgae = new SuperstructureState(0.,26.5, 25.8, 12.7);
        /*(x )*/processorAlgae = new SuperstructureState(0.,0.2,6.1,3.54);

        climb1 = new SuperstructureState(0., 1., 1., 0.);
        climb2 = new SuperstructureState(0., 1., 1., 0.);
        climb3 = new SuperstructureState(0., 0., 1., 0.);
       
    }

}
