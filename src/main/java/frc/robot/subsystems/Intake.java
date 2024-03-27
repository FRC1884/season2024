package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotMap.IntakeMap;

import java.util.Optional;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;

    }

    public enum IntakeDirection {
        FORWARD, REVERSE, STOPPED
    }

    private final Optional<CANSparkMax> intake;

    private IntakeDirection direction = IntakeDirection.STOPPED;

    private Intake() {
        super();

        if (Config.Subsystems.INTAKE_ENABLED) {
            intake = Optional.of(new CANSparkMax(IntakeMap.INTAKE_ID, MotorType.kBrushless));

            var motor = intake.get();

            motor.restoreFactoryDefaults();
            motor.setInverted(false);
            motor.setSmartCurrentLimit(20);
            motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
            motor.burnFlash();
        } else {
            intake = Optional.empty();
        }
    }

    private void setSpeed(double intakeSpeed) {
        intake.ifPresent(canSparkMax -> canSparkMax.set(intakeSpeed));
    }

    /**
     * Sets the intake state to the given direction
     *
     * @param direction the direction to set the intake to
     */
    public void setIntakeState(IntakeDirection direction) {
        if (direction == IntakeDirection.FORWARD) {
            this.direction = IntakeDirection.FORWARD;
        } else if (direction == IntakeDirection.STOPPED) {
            this.direction = IntakeDirection.STOPPED;
        } else if (direction == IntakeDirection.REVERSE) {
            this.direction = IntakeDirection.REVERSE;
        }
    }

    @Override
    public void periodic() {
        if (direction == IntakeDirection.FORWARD) {
            setSpeed(IntakeMap.INTAKE_FORWARD_SPEED);
        } else if (direction == IntakeDirection.REVERSE) {
            setSpeed(IntakeMap.INTAKE_REVERSE_SPEED);
        } else {
            setSpeed(0);
        }
    }

    public void toggleIntake(boolean isOn) {
        if (isOn) direction = IntakeDirection.FORWARD;
        else direction = IntakeDirection.STOPPED;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("intake on", () -> (direction == IntakeDirection.FORWARD), this::toggleIntake);
        builder.addStringProperty("status", () -> direction.toString(), (x) -> {});
    }
}
