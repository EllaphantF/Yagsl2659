package frc.robot.util.io;

import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.PathPlannerAutos.Auto;

public class Dashboard {
    private final SendableChooser<Auto> autoChooser;
    private static Dashboard INSTANCE;

    public static Dashboard getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Dashboard();
        }

        return INSTANCE;
    }
    
    private Dashboard() {

        autoChooser = new SendableChooser<Auto>();
        for (Auto auto : Auto.values()) {
            autoChooser.addOption(auto.name(), auto);
        }
        autoChooser.setDefaultOption(Auto.NO_AUTO.name(), Auto.NO_AUTO);
        SmartDashboard.putData("Auto", autoChooser);

    }

    public Auto getAuto() {
        return autoChooser.getSelected();
    }

} 
