package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

/**
 * An intake subsystem that utilizes sensor input.
 */
public class ExampleIntake extends SubsystemBase {
    private static ExampleIntake instance;

    public static ExampleIntake getInstance() {
        if(instance == null) instance = new ExampleIntake();
        return instance;
    }

    private CANSparkMax motor;

    /**
     * A Command that runs the intake at some power unless a note has been collected
     * and is in the robot.
     * @param objectIn Whether a note is currently in the robot.
     *                 This will be based on a sensor's input in the future,
     *                 however it is currently a BooleanSupplier for a type placeholder.
     * @return A Command that dictates the correct motor behavior
     * based on whether the note is in the robot
     */
    public Command run(BooleanSupplier objectIn) {
        return new ConditionalCommand(
                new RunCommand(() -> motor.set(0.0), this),
                new RunCommand(() -> motor.set(1.0), this),
                objectIn
        );
    }
}
