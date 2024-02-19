package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotMap.LEDMap;
import frc.robot.util.RunForSecondsCommand;

import java.util.function.DoubleSupplier;

import static frc.robot.RobotMap.LEDMap.NUMBER_LEDS;

public class AddressableLEDLights extends SubsystemBase {
    private static AddressableLEDLights instance;
    public static AddressableLEDLights getInstance() {
        if(instance == null) instance = new AddressableLEDLights();
        return instance;
    }

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue = 0;
    private int currLED, value = 255, direction = -1;

    private AddressableLEDLights() {
        m_led = new AddressableLED(LEDMap.BLINKIN_PWM_PORT);
        m_led.setLength(NUMBER_LEDS);
        m_ledBuffer = new AddressableLEDBuffer(NUMBER_LEDS);
        m_led.setData(m_ledBuffer);
        m_led.start();
        setDefaultCommand(setRainbow());
    }

    public Command setRedGreen(DoubleSupplier confidence) {
        return new RunCommand(
            ()->{
                for(int i = 0; i < NUMBER_LEDS; i++) {
                    m_ledBuffer.setHSV(
                        i,
                        (int) (confidence.getAsDouble() * 120),
                        255,
                        255
                    );
                }
                m_rainbowFirstPixelHue++;
                m_rainbowFirstPixelHue %= 180;
                m_led.setData(m_ledBuffer);
                m_led.start();
            }
        , this);
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

    public Command setRainbow() {
        return new RepeatCommand(new InstantCommand(() -> {
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
        }, this));
    }

    public Command setPhaseInOut(int hue) {
        return new RepeatCommand(new InstantCommand(() -> {
            for(int i = 0; i < NUMBER_LEDS; i++) {
                m_ledBuffer.setHSV(i, hue, 255, value);
            }

            value += direction;

            switch (value) {
                case 0 -> direction = 1;
                case 255 -> direction = -1;
            }

        }, this));
    }

    public Command setDoubleChase(Color colorOne, Color colorTwo) {
        return new RepeatCommand(new InstantCommand(()->{
            for(int i = 0; i < NUMBER_LEDS; i++) {
                m_ledBuffer.setLED(i, (currLED + i) % 2 == 0 ? colorOne: colorTwo);
            }
            currLED += 1;
            m_led.setData(m_ledBuffer);
            m_led.start();
        }).andThen(new WaitCommand(0.1)));
    }

    public Command setColorCommand(Color color) {
        return new RunCommand(() -> setColor(color), this);
    }

    public Command setToAllianceColorCommand() {
        return setColorCommand(DriverStation.getAlliance().get().equals(Alliance.Red) ? Color.kRed : Color.kBlue);
    }

    public Command disableCommand() {
        return setColorCommand(Color.kBlack);
    }
}