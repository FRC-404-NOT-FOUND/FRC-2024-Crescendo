package com.argsrobotics.lib.led.pattern;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import com.argsrobotics.lib.led.LEDStrip;

/**
 * A pattern that can be applied to an LED strip.
 * @see com.argsrobotics.lib.led.LEDStrip
 */
public interface LEDPattern {
    /**
     * Updates the LED buffer with the pattern.
     * @param buffer The LED buffer to update.
     * @param strip The strip to update the buffer with.
     */
    void updateBuffer(AddressableLEDBuffer buffer, LEDStrip strip);
}
