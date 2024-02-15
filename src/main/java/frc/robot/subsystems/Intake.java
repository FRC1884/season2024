package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.Subsystems;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public static enum IntakeDirection {
        FORWARD, REVERSE, STOPPED
    }

    public static enum IntakeStatus {
        EMPTY, LOADED
    }

    private CANSparkMax intake, feeder;
    private Optional<DigitalInput> intakeSensor;

    private IntakeStatus status = IntakeStatus.EMPTY;
    private IntakeDirection direction = IntakeDirection.STOPPED;

    private Intake() {
        setName("Intake");
        intake = new CANSparkMax(IntakeMap.INTAKE, MotorType.kBrushless);
        feeder = new CANSparkMax(IntakeMap.FEEDER, MotorType.kBrushless);

        intake.restoreFactoryDefaults();
        feeder.restoreFactoryDefaults();

        intake.setInverted(true);
        feeder.setInverted(true);

        intake.setSmartCurrentLimit(30);
        feeder.setSmartCurrentLimit(20);

        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        feeder.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intake.burnFlash();
        feeder.burnFlash();

        if (Subsystems.Intake.SENSOR_ENABLED) {
            intakeSensor = Optional.of(new DigitalInput(IntakeMap.SENSOR));
        } else {
            intakeSensor = Optional.empty();
        }

        var tab = Shuffleboard.getTab("Intake");

        // tab.add("intake motor", intake);
        // tab.add("feeder motor", feeder);

        if (intakeSensor.isPresent()) {
            tab.add("sensor", intakeSensor.get());
        }

        tab.addString("status", () -> status.toString());
        tab.addString("direction", () -> direction.toString());
    }

    private void setSpeed(double intakeSpeed, double feederSpeed) {
        intake.set(intakeSpeed);
        feeder.set(feederSpeed);
    }

    /**
     * Sets the intake state to the given direction
     * 
     * @param direction the direction to set the intake to
     * @return the command to set the intake state
     */
    public Command setIntakeState(IntakeDirection direction) {
        System.out.println("Intaking :)");
        if (direction == IntakeDirection.FORWARD) {
            return new InstantCommand(() -> this.direction = IntakeDirection.FORWARD);
        } else if (direction == IntakeDirection.REVERSE) {
            return new InstantCommand(() -> this.direction = IntakeDirection.REVERSE);
        } else {
            return new InstantCommand(() -> this.direction = IntakeDirection.STOPPED);
        }
    }

    public Command intakeUntilLoadedCommand() {
        return new FunctionalCommand(
                () -> direction = IntakeDirection.FORWARD,
                () -> {
                },
                (interrupt) -> {
                    if (!interrupt)
                        direction = IntakeDirection.STOPPED;
                },
                () -> status == IntakeStatus.LOADED,
                this);
    }

    @Override
    public void periodic() {
        if (intakeSensor.isPresent()) {
            if (!intakeSensor.get().get()) {
                status = IntakeStatus.EMPTY;
            } else {
                status = IntakeStatus.LOADED;
            }
        }

        if (direction == IntakeDirection.FORWARD) {
            setSpeed(IntakeMap.INTAKE_FORWARD_SPEED, IntakeMap.FEEDER_FORWARD_SPEED);
        } else if (direction == IntakeDirection.REVERSE) {
            setSpeed(IntakeMap.INTAKE_REVERSE_SPEED, IntakeMap.FEEDER_REVERSE_SPEED);
        } else {
            setSpeed(0, 0);
        }
    }
}
