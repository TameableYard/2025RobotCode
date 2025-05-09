package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase{
    private final Color titansRed = new Color(255, 1, 2);// 192 35 50     192 35 2
    private final Color titansDark = new Color(255, 0, 1);
    private final Color Black = Color.kBlack;
    private final Color White = Color.kWhite;
    private final Color titansLight = new Color(255, 2, 3);
    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(32);
    private final AddressableLEDBufferView m_frontLedBufferView = new AddressableLEDBufferView(m_ledBuffer, 0, 15);
    private final AddressableLEDBufferView m_backLedBufferView = new AddressableLEDBufferView(m_ledBuffer, 16, 31).reversed();
    private final LEDPattern m_test = LEDPattern.rainbow(255, 128);
    private final LEDPattern m_seizure = LEDPattern.gradient(GradientType.kContinuous, Black, White);
    private final LEDPattern m_red = LEDPattern.gradient(GradientType.kContinuous, titansRed, titansLight, titansDark);
    //private final LEDPattern m_red = LEDPattern.solid(titansRed);
    private final Distance kLEDSpacing = Meters.of(1/62.5);
    private final LEDPattern seizurePattern = m_seizure.scrollAtAbsoluteSpeed(MetersPerSecond.of(5), kLEDSpacing);
    private final LEDPattern rainbowPattern = m_test.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), kLEDSpacing);
    private final LEDPattern redPattern = m_red.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), kLEDSpacing);
    public LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    @Override
    public void periodic() {
        redPattern.applyTo(m_frontLedBufferView);
        redPattern.applyTo(m_backLedBufferView);
        m_led.setData(m_ledBuffer);
    }
}
