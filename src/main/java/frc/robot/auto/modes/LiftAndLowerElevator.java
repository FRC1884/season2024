package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ExampleElevator;
import frc.robot.subsystems.ExampleElevator.Setpoint;

public class LiftAndLowerElevator extends SequentialCommandGroup {
  public LiftAndLowerElevator() {
    var elevator = ExampleElevator.getInstance();

    addCommands(
        new SequentialCommandGroup(
            elevator.moveElevatorCommand(Setpoint.STATE_3),
            new WaitCommand(5.0),
            elevator.moveElevatorCommand(Setpoint.STATE_1)));
  }
}
