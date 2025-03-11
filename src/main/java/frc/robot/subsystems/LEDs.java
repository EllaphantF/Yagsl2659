package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.SuperstructureSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends SubsystemBase{
    final AddressableLED m_LED = new AddressableLED(9);

    final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(171);

    public final SuperstructureSubsystem m_SuperstructureSubsystem = new SuperstructureSubsystem();

    // public int lightMode = 0;

public LEDs(){
    m_LED.setLength(m_ledBuffer.getLength());

    // Set the data
    m_LED.setData(m_ledBuffer);
    m_LED.start();
        
}

/**
 * Sets the light mode
 * 0 = red
 * 1 = white
 * 2 = pink
 * 3 = orange
 * 4 = purple
 * 5 = red
 * 6 = green
 * 7 = green - bright
 * 8 = dark red
 * 9 = blue
 * 10 = cyan
 * 11 = yellow
 * 12 = dark blue
 * 13 = some magenta bs
 * 
 * @param mode
 */
/* public void setLightMode(int mode){
    lightMode = mode;
} */

@Override
public void periodic() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the RGB values for red
        //m_ledBuffer.setRGB(i, 255, 255  , 255);
        if (m_SuperstructureSubsystem.lightMode == 0) {
           m_ledBuffer.setRGB(i, 255, 0 , 0); 
        } else if (m_SuperstructureSubsystem.lightMode == 1){
            m_ledBuffer.setRGB(i, 247,247,247);
        } else if (m_SuperstructureSubsystem.lightMode == 2){
            m_ledBuffer.setRGB(i, 255,0,213);
        } else if (m_SuperstructureSubsystem.lightMode == 3){
            m_ledBuffer.setRGB(i, 255,100,10);
        } else if (m_SuperstructureSubsystem.lightMode == 4){
            m_ledBuffer.setRGB(i, 141,3,183);
        } else if (m_SuperstructureSubsystem.lightMode == 5) {
            m_ledBuffer.setRGB(i, 255,30,0);
        } else if (m_SuperstructureSubsystem.lightMode == 6) {
            m_ledBuffer.setRGB(i, 100,255,12);
        } else if (m_SuperstructureSubsystem.lightMode == 7) {
            m_ledBuffer.setRGB(i, 10,255,10);
        } else if (m_SuperstructureSubsystem.lightMode == 8) {
            m_ledBuffer.setRGB(i, 120,0,0);
        } else if (m_SuperstructureSubsystem.lightMode == 9) {
            m_ledBuffer.setRGB(i, 55, 94, 220);
        } else if (m_SuperstructureSubsystem.lightMode == 10) {
            m_ledBuffer.setRGB(i, 55, 209, 0);
        } else if (m_SuperstructureSubsystem.lightMode == 11) {
            m_ledBuffer.setRGB(i, 232, 236, 104);
        } else if (m_SuperstructureSubsystem.lightMode == 12) {
            m_ledBuffer.setRGB(i, 66, 43, 158);
        } else if (m_SuperstructureSubsystem.lightMode == 13) {
            m_ledBuffer.setRGB(i, 158, 43, 120);
        }
     }
     //System.out.println("LEDs BEING SET");
     m_LED.setData(m_ledBuffer);
}


}