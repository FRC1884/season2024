package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDLights extends SubsystemBase {
    
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
    // Solid Colors
    public void solid_hotPink() {
        set(0.57);
    }

    public void solid_darkRed() {
        set(0.59);
    }

    public void solid_red() {
        set(0.61);
    }

    public void solid_redOrange() {
        set(0.63);
    }

    public void solid_orange() {
        set(0.65);
    }

    public void solid_gold() {
        set(0.67);
    }

    public void solid_yellow() {
        set(0.69);
    }

    public void solid_lawnGreen() {
        set(0.71);
    }

    public void solid_lime() {
        set(0.73);
    }

    public void solid_darkGreen() {
        set(0.75);
    }

    public void solid_green() {
        set(0.77);
    }

    public void solid_blueGreen() {
        set(0.79);
    }

    public void solid_aqua() {
        set(0.81);
    }

    public void solid_skyBlue() {
        set(0.83);
    }

    public void solid_darkBlue() {
        set(0.85);
    }

    public void solid_blue() {
        set(0.87);
    }

    public void solid_blueViolet() {
        set(0.89);
    }

    public void solid_violet() {
        set(0.91);
    }

    public void solid_white() {
        set(0.93);
    }

    public void solid_gray() {
        set(0.95);
    }

    public void solid_darkGray() {
        set(0.97);
    }

    public void solid_black() {
        set(0.99);
    }

    // Patterns

    public void pattern_rainbow() {
        set(-0.99);
    }

    public void patternColorWave_lava() {
        set(-0.39);
    }

    public void patternColorWave_ocean() {
        set(-0.41);
    }

    public void pattern_heartbeatRed() {
        set(-0.25);
    }

    public void pattern_heartbeatBlue() {
        set(-0.23);
    }

    public void allianceColor() {
        boolean isRed = NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("IsRedAlliance").getBoolean(true);
        if (isRed == true){
            set(-0.01);
        } else {
            set(0.19);
        }
    }

    public Command setColorRed() {
        return new InstantCommand (() -> solid_red());
    }

    public Command setColorOrange() {
        return new InstantCommand (() -> solid_orange());
    }

    public Command setColorYellow() {
        return new InstantCommand (() -> solid_yellow());
    }

    public Command setColorGreen() {
        return new InstantCommand (() -> solid_green());
    }

    public Command setColorBlue() {
        return new InstantCommand (() -> solid_blue());
    }

    public Command setColorViolet() {
        return new InstantCommand (() -> solid_violet());
    }

    public Command setPatternColorWaveOcean() {
        return new InstantCommand (() -> patternColorWave_ocean());
    }

    public Command setPatternColorWaveLava() {
        return new InstantCommand (() -> patternColorWave_lava());
    }

    public Command setPatternRainbow() {
        return new InstantCommand (() -> pattern_heartbeatBlue());
    }
}
    
