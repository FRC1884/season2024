package frc.robot.auto.selector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoModeSelector {
  private static AutoModeSelector instance;

  public static AutoModeSelector getInstance() {
    if (instance == null) {
      instance = new AutoModeSelector();
    }
    return instance;
  }

  private final SendableChooser<Command> modeChooser;

  private AutoModeSelector() {
    modeChooser = new SendableChooser<>();
  }

  public SendableChooser<Command> getChooser() {
    return modeChooser;
  }

}
