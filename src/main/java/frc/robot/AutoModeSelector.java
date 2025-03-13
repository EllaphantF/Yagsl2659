package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoModeSelector {

    enum AutonMode{
        TEST_AUTO, SCORE_BL4
    }
    
    private AutonMode mCachedAutoMode = null;

    private static SendableChooser<AutonMode> mAutoMode;

    private static String autoChoiceReturn;

    public AutoModeSelector(){
        mAutoMode = new SendableChooser<>();
        mAutoMode.setDefaultOption("TEST AUTO", AutonMode.TEST_AUTO);
        mAutoMode.addOption("SCORE B L4", AutonMode.SCORE_BL4);
        SmartDashboard.putData("Auto Mode", mAutoMode);
    }

    public void updateModeCreator(){
        AutonMode desiredMode = mAutoMode.getSelected();
        if (mCachedAutoMode != desiredMode){
            System.out.println("Auto selection changed, updating creator: desiredMode ->" + desiredMode.name());
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
        }
        
        return 0;
    }

    public void outputToSmartDashboard(){
        SmartDashboard.putString("Auto Mode Selected", mCachedAutoMode.name());
    }

    public void reset(){
        mCachedAutoMode = null;
    }
}
