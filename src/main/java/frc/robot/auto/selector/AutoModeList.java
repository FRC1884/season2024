package frc.robot.auto.selector;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.*;

public enum AutoModeList {
  DO_NOTHING(new DoNothing()),
  TEST_PATH(new DoNothing());

  private final SequentialCommandGroup autoCommand;

  AutoModeList(SequentialCommandGroup autoCommand) {
    this.autoCommand = autoCommand;
  }

  public SequentialCommandGroup getAuto() {
    return autoCommand;
  }
}
