package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.RunForSecondsCommand;

import java.util.stream.IntStream;

import static frc.robot.util.BlinkinUtils.ColorPatterns.BLACK;

public class AddressableLEDLights extends SubsystemBase {
    private static AddressableLEDLights instance;
    public static AddressableLEDLights getInstance() {
        if(instance == null) instance = new AddressableLEDLights();
        return instance;
    }

    private AddressableLED m_blinkin;
    private AddressableLEDBuffer m_blinkinBuffer;

    public AddressableLEDLights() {
        m_blinkin = new AddressableLED(RobotMap.LEDMap.BLINKIN_PWM_PORT);
        m_blinkin.setLength(RobotMap.LEDMap.NUMBER_LEDS);
        m_blinkinBuffer = new AddressableLEDBuffer(RobotMap.LEDMap.NUMBER_LEDS);
        m_blinkin.setData(m_blinkinBuffer);
    }

    private void setColor(Color color) {
        IntStream.range(0, m_blinkinBuffer.getLength()).forEachOrdered(i -> m_blinkinBuffer.setLED(i, color));
    }

    public void setToAllianceColor() {
        IntStream.range(0, m_blinkinBuffer.getLength()).forEachOrdered(i -> {
            if (DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                m_blinkinBuffer.setLED(i, Color.kRed);
            else m_blinkinBuffer.setLED(i, Color.kBlue);
        });
    }

    public Command setColorCommand(Color color) {
        return new InstantCommand(() -> setColor(color));
    }

    public Command setColorForSecondsCommand(Color color) {
        return new RunForSecondsCommand(3.0, setColorCommand(color))
                .andThen(setColorCommand(Color.kBlack));
    }

    public Command disableCommand() {
        return new InstantCommand(() -> setColor(Color.kBlack));
    }
}