package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DoNothing extends SequentialCommandGroup {
  public DoNothing() {
    addCommands(new WaitCommand(15.0));
  }
}
