 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDUtility extends SubsystemBase {
  AddressableLED addressableLED;
  int overallLength = 0;
  ArrayList<LEDStrip> ledStrips = new ArrayList<>();
  AddressableLEDBuffer filler = new AddressableLEDBuffer(0);

  /** Creates a new LEDUtility. */
  public LEDUtility(int _port) {
    addressableLED = new AddressableLED(_port);
    addressableLED.start();
  }

  public void addStrip(LEDStrip _strip) {
    ledStrips.add(_strip);
    overallLength = overallLength + _strip.getLength();
    setLength();
  }

  public LEDStrip getStrip(int index) {
    return ledStrips.get(index);
  }

  private void setLength() {
    filler = new AddressableLEDBuffer(overallLength);
    addressableLED.setLength(overallLength);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //LEDEffects.setRainbow(this.getStrip(0));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(1));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(2));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(3));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(4));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(5));//, Color.kGreen, 5);
    //LEDEffects.setRainbow(this.getStrip(6));//, Color.kGreen, 5);
    ledStrips.forEach(strip -> {
      for(int i = strip.getStart(); i <= strip.getStop(); i++) {
        filler.setLED(i, strip.getIndex(i - strip.getStart()));
      }
    });
    addressableLED.setData(filler);
    addressableLED.start();
  }
}
