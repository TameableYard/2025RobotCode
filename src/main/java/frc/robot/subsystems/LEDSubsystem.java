package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private final LEDPattern m_test = LEDPattern.rainbow(255, 128);
    private final Distance kLEDSpacing = Meters.of()
    public LEDSubsystem() {
        m_led.setData(m_ledBuffer);
    }
}
