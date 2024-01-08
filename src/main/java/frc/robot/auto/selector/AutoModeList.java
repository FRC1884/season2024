package frc.robot.auto.selector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.*;

public enum AutoModeList {
  DO_NOTHING(new DoNothing()),
  LIFT_AND_LOWER_ELEVATOR(new LiftAndLowerElevator()),
  TESTPATH(new TestPath());

  private final SequentialCommandGroup autoCommand;

  AutoModeList(SequentialCommandGroup autoCommand) {
    this.autoCommand = autoCommand;
  }

  public SequentialCommandGroup getAuto() {
    return autoCommand;
  }
}
