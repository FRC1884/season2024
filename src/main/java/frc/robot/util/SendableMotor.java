package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * The {@link Sendable} implementation for motors that allows us to send an entire motor
 * as a package to Shuffleboard or Elastic.
 * <br><br>
 * <b>For this to work, you MUST run the program in Test Mode on the Driver Station!</b>
 */
public class SendableMotor implements Sendable {
    private CANSparkBase m_motorController;
    private SparkPIDController closedLoopController;

    private double m_setpoint;
    private boolean closedLoopEnabled, openLoopEnabled;

    public SendableMotor(CANSparkBase motorController) {
        m_motorController = motorController;
        closedLoopController = motorController.getPIDController();
    }

    void enable(boolean enabled) {
        openLoopEnabled = enabled;
    }

    void enablePID(boolean enabled) {
        closedLoopEnabled = enabled;
    }

    void setSpeed(double speed) {
        if(openLoopEnabled) m_motorController.set(speed);
    }

    void setP(double p) {
        if(openLoopEnabled && closedLoopEnabled) closedLoopController.setP(p);
    }

    void setI(double i) {
        if(openLoopEnabled && closedLoopEnabled) closedLoopController.setI(i);
    }

    void setD(double d) {
        if(openLoopEnabled && closedLoopEnabled) closedLoopController.setD(d);
    }

    void setFF(double ff) {
        if(openLoopEnabled && closedLoopEnabled) closedLoopController.setFF(ff);
    }

    void setIZone(double iZone) {
        if(openLoopEnabled && closedLoopEnabled) closedLoopController.setIZone(iZone);
    }

    void setMaxVel(double vel) {
        if(openLoopEnabled && closedLoopEnabled)
            closedLoopController.setSmartMotionMaxVelocity(vel, m_motorController.getDeviceId());
    }

    void setMaxAccel(double accel) {
        if(openLoopEnabled && closedLoopEnabled)
            closedLoopController.setSmartMotionMaxAccel(accel, m_motorController.getDeviceId());
    }

    void setTolerance(double tolerance) {
        if(openLoopEnabled && closedLoopEnabled)
            m_motorController.getPIDController().setSmartMotionAllowedClosedLoopError(tolerance, m_motorController.getDeviceId());
    }

    void setSetpoint(double setpoint) {
        if(openLoopEnabled && closedLoopEnabled) {
            m_setpoint = setpoint;
            closedLoopController.setReference(setpoint, ControlType.kSmartMotion);
        }
    }

    // Adds a bunch of control properties on shuffleboard
    // Do changes on the board affect behavior?
    // Do I need to goof around in periodic?
    // Only time will tell...
    @Override
    public void initSendable(SendableBuilder builder) {
        SparkPIDController closedLoopController = m_motorController.getPIDController();
        builder.setActuator(true);

        builder.addBooleanProperty("Enabled", () -> openLoopEnabled, this::enable);
        builder.addBooleanProperty("PID Enabled", () -> closedLoopEnabled, this::enablePID);

        builder.addDoubleProperty("Speed", m_motorController::get, this::setSpeed);

        builder.addDoubleProperty("Setpoint",
                () -> m_setpoint,
                this::setSetpoint);

        builder.addDoubleProperty("P", closedLoopController::getP, this::setP);
        builder.addDoubleProperty("I", closedLoopController::getI, this::setI);
        builder.addDoubleProperty("D", closedLoopController::getD, this::setD);
        builder.addDoubleProperty("FF", closedLoopController::getFF, this::setFF);
        builder.addDoubleProperty("IZone", closedLoopController::getIZone, this::setIZone);

        int id = m_motorController.getDeviceId();

        builder.addDoubleProperty("MaxVel",
                () -> closedLoopController.getSmartMotionMaxVelocity(id),
                this::setMaxVel);
        builder.addDoubleProperty("MaxAccel",
                () -> closedLoopController.getSmartMotionMaxAccel(id),
                this::setMaxAccel);
        builder.addDoubleProperty("AllowedErr",
                () -> closedLoopController.getSmartMotionAllowedClosedLoopError(id),
                this::setTolerance);
    }
}