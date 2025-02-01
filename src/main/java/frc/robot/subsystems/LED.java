package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED extends SubsystemBase {
  private final int PWMPORT = 1;
  private final int LEDLENGTH = 150;
  private final AddressableLED m_led = new AddressableLED(PWMPORT);
  private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDLENGTH);
  private int m_rainbowFirstPixelHue;
  private final Timer blink = new Timer();
  private double lastChange;
  private boolean on=true;

  public LED() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    blink.start();
  }

  @Override
  public void periodic() {

  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }


  public void setAll(Color color) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setLED(i, color);
    }
    m_led.setData(m_ledBuffer);

  }

  public void setAllBlink(Color color, Double sec) {

    double timestamp = Timer.getFPGATimestamp();
    if(timestamp - lastChange > 0.15){
      on = !on;
      lastChange = timestamp;
    }
    if (on){
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
      }

      m_led.setData(m_ledBuffer);
    }else{
      color = Color.kBlack;
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setLED(i, color);
      }
      m_led.setData(m_ledBuffer);
      
    }
  }

}