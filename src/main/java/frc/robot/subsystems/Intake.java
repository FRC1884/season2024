package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public static Intake getInstance() {
        if(instance == null) {
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

    private CANSparkMax intake;
    

    private IntakeStatus status = IntakeStatus.EMPTY;
    private IntakeDirection direction = IntakeDirection.STOPPED;

    private Intake() {
        setName("Intake");
        intake = new CANSparkMax(IntakeMap.INTAKE_ID, MotorType.kBrushless);

        intake.restoreFactoryDefaults();
        intake.setInverted(false);
        intake.setSmartCurrentLimit(20);
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake.burnFlash();

        var tab = Shuffleboard.getTab("Intake");

        tab.addString("status", () -> status.toString());
        tab.addString("direction", () -> direction.toString());
    }

    private void setSpeed(double intakeSpeed) {
        intake.set(intakeSpeed);
    }

    /**
     * Sets the intake state to the given direction
     * 
     * @param direction the direction to set the intake to
     * @return the command to set the intake state
     */
    public void setIntakeState(IntakeDirection direction) {
        if (direction == IntakeDirection.FORWARD) {
            this.direction = IntakeDirection.FORWARD;
        } 
        else if(direction == IntakeDirection.STOPPED){
            this.direction = IntakeDirection.STOPPED;
        }
        else if(direction == IntakeDirection.REVERSE){
            this.direction = IntakeDirection.REVERSE;
        }
        //else if (direction == IntakeDirection.REVERSE) {
        //     return new InstantCommand(() -> this.direction = (this.direction == IntakeDirection.STOPPED) ? IntakeDirection.REVERSE : IntakeDirection.STOPPED);
        // } else {
        //     return new InstantCommand(() -> this.direction = IntakeDirection.STOPPED);
        // }
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

        if (direction == IntakeDirection.FORWARD) {
            setSpeed(IntakeMap.INTAKE_FORWARD_SPEED);
        } else if (direction == IntakeDirection.REVERSE) {
            setSpeed(IntakeMap.INTAKE_REVERSE_SPEED);
        } else {
            setSpeed(0);
        }
    }

    public void setIntake(boolean isOn){
        if(isOn) direction = IntakeDirection.FORWARD;
        else direction = IntakeDirection.STOPPED;
    }


    @Override
    public void initSendable(SendableBuilder builder){
        builder.addBooleanProperty("Intake On", () -> (direction==IntakeDirection.FORWARD), this::setIntake);
    }
    
}
