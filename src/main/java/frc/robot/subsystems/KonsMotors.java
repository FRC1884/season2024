package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

/** An intake subsystem that utilizes sensor input. */
public class KonsMotors extends SubsystemBase {
  private static KonsMotors instance;
  private static boolean running;

  public static KonsMotors getInstance() {
    if (instance == null) instance = new KonsMotors();
    return instance;
  }

  private CANSparkMax motorOne;
  private CANSparkMax motorTwo;

  public KonsMotors() {
motorTwo.follow(motorOne, true);
motorOne.set(0);
running = false;
  }

  /**
   * A Command that runs the intake at some power unless a note has been collected and is in the
   * robot.
   *
   * @param objectIn Whether a note is currently in the robot. This will be based on a sensor's
   *     input in the future, however it is currently a BooleanSupplier for a type placeholder.
   * @return A Command that dictates the correct motor behavior based on whether the note is in the
   *     robot
   */
  public Command run() {
    return new InstantCommand( () -> {motorOne.set(1.0); running = true;});
  }
  public Command kill() {
    return new InstantCommand( () -> {motorOne.set(0); running = false;});
  }
  public boolean isRunning() {
    return running;
  }
}
