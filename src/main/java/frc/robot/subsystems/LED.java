package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LED extends SubsystemBase {

  private final AddressableLED led = new AddressableLED(LEDConstants.PWM_PORT);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);
  private int rainbowFirstPixelHue;
  private final Timer blink = new Timer();
  private double lastChange;
  private boolean on=true;

  public LED() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
    blink.start();
  }

  @Override
  public void periodic() {

  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by 1 to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    // Check bounds
    rainbowFirstPixelHue %= 180;
    led.setData(ledBuffer);
  }


  public void setAll(Color color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
    led.setData(ledBuffer);

  }

  public void setAllBlink(Color color, Double sec) {

    double timestamp = Timer.getFPGATimestamp();
    if(timestamp - lastChange > sec){
      on = !on;
      lastChange = timestamp;
    }
    if (on){
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color);
      }

      led.setData(ledBuffer);
    }else{
      color = Color.kBlack;
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setLED(i, color);
      }
      led.setData(ledBuffer);
    }
  }

}