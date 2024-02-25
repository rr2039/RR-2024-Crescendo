 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDUtility extends SubsystemBase {
  AddressableLED addressableLED;
  int overallLength = 0;
  ArrayList<LEDStrip> ledStrips = new ArrayList<>();

  /** Creates a new LEDUtility. */
  public LEDUtility(int _port) {
    addressableLED = new AddressableLED(_port);
  }

  public void addStrip(LEDStrip _strip) {
    ledStrips.add(_strip);
    overallLength += _strip.getLength();
    setLength();
  }

  public LEDStrip getStrip(int index) {
    return ledStrips.get(index);
  }

  private void setLength() {
    addressableLED.setLength(overallLength);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    AddressableLEDBuffer filler = new AddressableLEDBuffer(overallLength);
    ledStrips.forEach(strip -> {
      for(int i = strip.getStart()-1; i <= strip.getStop()-1; i++) {
        filler.setLED(i, strip.getIndex(i));
      }
    });
  }
}
