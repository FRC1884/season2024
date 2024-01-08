package frc.robot.auto.selector;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.modes.TestPath;

public class AutoModeSelector {
  private static AutoModeSelector instance;

  public static AutoModeSelector getInstance() {
    if (instance == null) {
      instance = new AutoModeSelector();
    }
    return instance;
  }

  private final TestPath testPath = new TestPath();

  private final SendableChooser<Command> modeChooserRed;
  private final SendableChooser<Command> modeChooserBlue;

  private AutoModeSelector() {
    modeChooserRed = new SendableChooser<>();
    modeChooserBlue = new SendableChooser<>();
    updateAutoModeSelector();
  }

  public void updateAutoModeSelector() {
    modeChooserRed.setDefaultOption("DO_NOTHING", AutoModeList.DO_NOTHING.getAuto());
    modeChooserBlue.setDefaultOption("DO_NOTHING", AutoModeList.DO_NOTHING.getAuto());
  }

  public SendableChooser<Command> getRedChooser() {
    return modeChooserRed;
  }

  public SendableChooser<Command> getBlueChooser() {
    return modeChooserBlue;
  }
}
