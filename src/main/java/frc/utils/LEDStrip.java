// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDStrip {
    int start;
    int stop;
    int helperPos = 0;
    boolean invert = false;
    AddressableLEDBuffer buffer;

    public LEDStrip(int _start, int _stop, boolean _invert) {
        start = _start;
        stop = _stop;
        invert = _invert;
        buffer = new AddressableLEDBuffer((stop - start) + 1);
    }

    public int getStart() {
        return start;
    }

    public int getStop() {
        return stop;
    }

    public int getHelperPos() {
        return helperPos;
    }

    public int getLength() {
        return buffer.getLength();
    }

    public void setStart(int _start) {
        start = _start;
    }

    public void setStop(int _stop) {
        stop = _stop;
    }

    public void setHelperPos(int _helperPos) {
        helperPos = _helperPos;
    }

    public void setBuffer(AddressableLEDBuffer _buffer) {
        buffer = _buffer;
    }

    public Color getIndex(int index) {
        return buffer.getLED(index);
    }
}
