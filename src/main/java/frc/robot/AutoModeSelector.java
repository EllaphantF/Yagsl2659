package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    enum AutonMode {
        TEST_AUTO, SCORE_BL4
    }

    private AutonMode mCachedAutoMode = null;

 //    private SendableChooser<DesiredMode> mModeChooser;
    private static SendableChooser<AutonMode> mAutoMode;
 //   private AutonMode autoModeReturn = null;
    private static String autoChoiceReturn;

    public AutoModeSelector() {
        mAutoMode = new SendableChooser<>();
        mAutoMode.setDefaultOption("TEST AUTO", AutonMode.TEST_AUTO); //ID: 1
        mAutoMode.addOption("SCORE B L4", AutonMode.SCORE_BL4); //ID: 2
        SmartDashboard.putData("Auto Mode", mAutoMode);
    }

    public void updateModeCreator() {
        AutonMode desiredMode = mAutoMode.getSelected();
         if (mCachedAutoMode != desiredMode ) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
          //  autoModeReturn = desiredMode;
        }
        mCachedAutoMode = desiredMode;
    }

    public static int returnAutoMode(){
       autoChoiceReturn = mAutoMode.getSelected().toString();
        SmartDashboard.putString(autoChoiceReturn, "autoChoiceReturn");
       switch (autoChoiceReturn) {
            case "TEST_AUTO": 
            return 1;

            case "SCORE_BL4":
            return 2;

            /* case "RED_4_PIECE":
            return 3;

            case "BLUE_AMP":
            return 4;

            case "RED_AMP":
            return 5;

            case "BLUE_SOURCE":
            return 6;

            case "RED_SOURCE":
            return 7;

            case "RED_MID_5":
            return 9;

            case "BLUE_MID_5":
            return 8; */
       }

       return 0;
    }

    public void outputToSmartDashboard() {
       // SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("Auto Mode Selected", mCachedAutoMode.name());
    }

    public void reset() {
        mCachedAutoMode = null;
    }
}