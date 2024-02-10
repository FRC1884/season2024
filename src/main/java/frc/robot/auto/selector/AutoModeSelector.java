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
    updateAutoModeSelector();

    for(AutoModeList auto : AutoModeList.values()){
      modeChooser.addOption(auto.name(), auto.getAuto());
    }
  }

  public void updateAutoModeSelector() {
    modeChooser.setDefaultOption("DO_NOTHING", AutoModeList.DO_NOTHING.getAuto());
  }

  public SendableChooser<Command> getChooser() {
    return modeChooser;
  }

}
