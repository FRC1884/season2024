package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.BlinkinUtils;
import frc.robot.util.RunForSecondsCommand;

import static frc.robot.util.BlinkinUtils.ColorPatterns.*;

public class PWMLEDLights extends SubsystemBase {
    private static PWMLEDLights instance;

    public static PWMLEDLights getInstance() {
        if(instance == null) instance = new PWMLEDLights();
        return instance;
    }

    private Spark m_blinkin;

    //@param pwmPort  The PWM port the Blinkin is connected to.
    public PWMLEDLights() {
      m_blinkin = new Spark(RobotMap.LEDMap.BLINKIN_PWM_PORT);
    }

    public void setColor(BlinkinUtils.ColorPatterns pattern) {
        setColor(pattern.get());
    }

    public void setColor(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
            // scaling the PWM signal to an integer to ensure that
            // the hundredths digit is odd before setting the color
            if (Math.round(val * 100 % 2) == 1) m_blinkin.set(val);
        } else System.out.println("PWM value out of range, argument ignored!");
    }

    /**
     * Sets the Blinkin controller based on the alliance color
     * from the Driver Station's network tether.
     * This will either use the FMS info at competition,
     * or the alliance station dropdown on the DS.
     */
    public void setToAllianceColor() {
        /*boolean isRed = NetworkTableInstance
                .getDefault()
                .getTable("FMSInfo")
                .getEntry("IsRedAlliance")
                .getBoolean(true);*/
        setColor(
                DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                        ? RED : BLUE);
    }

    public Command setColorCommand(double value) {
        return new InstantCommand(() -> setColor(value));
    }

    public Command setColorCommand(BlinkinUtils.ColorPatterns value) {
        return new InstantCommand(() -> setColor(value));
    }

    public Command setColorForSecondsCommand(double seconds, double value) {
        return new RunForSecondsCommand(seconds, setColorCommand(value))
                .andThen(new InstantCommand(() -> setColor(BLACK)));
    }

    public Command setColorForSecondsCommand(double seconds, BlinkinUtils.ColorPatterns value) {
        return new RunForSecondsCommand(seconds, setColorCommand(value))
                .andThen(new InstantCommand(() -> setColor(BLACK)));
    }

    public Command disableCommand() {
        return new InstantCommand(() -> setColor(BLACK));
    }
}