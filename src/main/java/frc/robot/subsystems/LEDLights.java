package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDLights extends SubsystemBase {
    
public enum ColorList {
    
    hotPink(0.57),
    darkRed(0.59), 
    red(0.61),
    redOrange(0.63),
    orange(0.65),
    gold(0.67),
    yellow(0.69),
    lawnGreen(0.71),
    lime(0.73),
    darkGreen(0.75),
    green(0.77),
    blueGreen(0.79),
    aqua(0.81),
    skyBlue(0.83),
    darkBlue(0.85),
    blue(0.87), 
    blueViolet(0.89),
    violet(0.91),
    white(0.93),
    gray(0.95),
    darkGrey(0.97),
    black(0.99),
    pattern_rainbow(-0.99),
    pattern_lava(-0.39),
    pattern_ocean(-0.41),
    pattern_heartbeatBlue(-0.23),
    pattern_heartbeatRed(-0.25);

    private final double colorValue;
    private ColorList(double color) {
        this.colorValue = color;
    } 
    public double get_colorValue() {
        return colorValue;
    }
}

    public static Spark m_blinkin = null;

    //@param pwmPort  The PWM port the Blinkin is connected to.
  public LEDLights(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
}

    public void set(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
        m_blinkin.set(val);
        }
    }

    public void allianceColor() {
        boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        if (isRed == true){
            set(-0.01);
        } else {
            set(0.19);
        }
    }

    public Command setColorCommand(ColorList color){
        return new InstantCommand(() -> set(color.get_colorValue()));
    }
}
    
