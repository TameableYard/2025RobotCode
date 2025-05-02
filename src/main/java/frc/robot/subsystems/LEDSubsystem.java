package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);
    private final LEDPattern m_test = LEDPattern.rainbow(255, 128);
    private final Distance kLEDSpacing = Meters.of(1/62.5);
    private final LEDPattern rainbowPattern = m_test.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25), kLEDSpacing);
    public LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    @Override
    public void periodic() {
        rainbowPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}
