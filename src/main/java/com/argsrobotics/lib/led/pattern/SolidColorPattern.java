package com.argsrobotics.lib.led.pattern;

import com.argsrobotics.lib.led.Color;
import com.argsrobotics.lib.led.LEDStrip;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * A solid color pattern that can be applied to an LED strip.
 *
 * @see com.argsrobotics.lib.led.LEDStrip
 */
public class SolidColorPattern implements LEDPattern {
  /**
   * Updates the LED buffer with the pattern.
   *
   * @param buffer The LED buffer to update.
   * @param strip The strip to update the buffer with.
   */
  @Override
  public void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip) {
    int offset = strip.getOffset();
    int length = strip.getLength();
    Color.HSV color = strip.getPrimaryColor().getHSV();

    for (int i = 0; i < length; i++) {
      buffer.setHSV(offset + i, color.hue(), color.saturation(), color.value());
    }
  }
}
