package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Config;
import frc.robot.RobotMap.LEDMap;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotMap.LEDMap.NUMBER_LEDS;

public class AddressableLEDLights extends SubsystemBase {
    private static AddressableLEDLights instance;
    public static AddressableLEDLights getInstance() {
        if(instance == null) instance = new AddressableLEDLights();
        return instance;
    }

    public enum LEDState {
        NO_NOTE,
        HAS_NOTE,
        AMPLIFY,
        COOP;
    }

    private LEDState state = LEDState.NO_NOTE;

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
    private int value = 255, direction = -1;

    private AddressableLEDLights() {
        m_led = new AddressableLED(LEDMap.BLINKIN_PWM_PORT);
        m_led.setLength(NUMBER_LEDS);
        m_ledBuffer = new AddressableLEDBuffer(NUMBER_LEDS);
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public LEDState getState() {
        return state;
    }

    private Command setNoteStatusState(BooleanSupplier beamBroken) {
        return Commands.runOnce(()-> {
            if (beamBroken.getAsBoolean()) this.state = LEDState.HAS_NOTE;
            else this.state = LEDState.NO_NOTE;
        });
    }

    public Command toggleAmplifyState(BooleanSupplier beamBroken) {
        return getState() == LEDState.AMPLIFY
                ? setNoteStatusState(beamBroken)
                : Commands.runOnce(() -> this.state = LEDState.AMPLIFY);
    }

    public Command toggleCoopState(BooleanSupplier beamBroken) {
        return getState() == LEDState.COOP
                ? setNoteStatusState(beamBroken)
                : Commands.runOnce(() -> this.state = LEDState.COOP);
    }

    public Command useState(Supplier<LEDState> state) {
        return Commands.select(
                Map.of(
                        LEDState.NO_NOTE, setColorCommand(Color.kRed),
                        LEDState.HAS_NOTE, setBlinking(Color.kGreen, Color.kBlack, 6.0),
                        LEDState.AMPLIFY, getAmplifyPattern(),
                        LEDState.COOP, getCoOpPattern()
                ),
                this::getState
        ).repeatedly();
    }

    private void setColor(Color color) {
        for(int i = 0; i < NUMBER_LEDS; i++) {
            m_ledBuffer.setRGB(
                i, 
                (int) color.green * 255, 
                (int) color.red * 255, 
                (int) color.blue * 255
            );
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private Command setRainbow() {
        return Commands.runOnce(() -> {
            // For every pixel
            for (var i = 0; i < NUMBER_LEDS; i++) {
                // Calculate the hue - hue is easier for rainbows because the color
                // shape is a circle so only one value needs to precess
                final var hue = (m_rainbowFirstPixelHue + (i * 180 / NUMBER_LEDS)) % 180;
                // Set the value
                m_ledBuffer.setHSV(i, hue, 255, 128);
            }
            // Increase by to make the rainbow "move"
            m_rainbowFirstPixelHue += 1;
            // Check bounds
            m_rainbowFirstPixelHue %= 180;
            m_led.setData(m_ledBuffer);
            m_led.start();
        }, this).repeatedly();
    }

    private Command getAmplifyPattern() {
        return setPhaseInOut(() -> Config.IS_ALLIANCE_RED)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private Command getCoOpPattern() {
        return setRainbow()
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private Command setBlinking(Color onColor, Color offColor, double frequency) {
        return Commands.repeatingSequence(
                setColorCommand(onColor).withTimeout(1.0 / frequency),
                setColorCommand(offColor).withTimeout(1.0 / frequency));
    }

    private Command setPhaseInOut(BooleanSupplier isRed) {
        return Commands.runOnce(() -> {
            for(int i = 0; i < NUMBER_LEDS; i++) {
                m_ledBuffer.setHSV(
                    i, 
                    isRed.getAsBoolean() ? 240 : 120, 
                    255, 
                    value
                );
            }

            value += 10 * direction;

            if(value <= 0) {
                value = 0;
                direction = 1;
            }
            if(value >= 255) {
                value = 255;

                direction = -1;
            }

            m_led.setData(m_ledBuffer);
            m_led.start();
        }, this).repeatedly().beforeStarting(() -> value = 255);
    }

    private Command setColorCommand(Color color) {
        return Commands.run(() -> setColor(color), this);
    }
    
}