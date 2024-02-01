package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Prototypes;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getResetButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getShootButton();

  private void registerPrototype() {
    Prototypes prototypes = Prototypes.getInstance();
    getResetButton().onTrue(prototypes.run(0.0, 0.0, 0.0, 0.0));
    getIntakeButton().onTrue(prototypes.run(0.0, 0.0, 1.0, 1.0));
    getShootButton().onTrue(prototypes.run(1.0, 1.0, 0.0, 0.0));
  }

  @Override
  public void registerCommands() {
    registerPrototype();
  }
}
