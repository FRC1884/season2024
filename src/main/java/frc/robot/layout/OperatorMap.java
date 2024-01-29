package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.OneMotorIntake;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getRunMotorButton();

  abstract JoystickButton getStopMotorButton();

  private void registerIntake() {
    if (ExampleConfig.Subsystems.INTAKE_ENABLED) {
      var intake = OneMotorIntake.getInstance();
      getRunMotorButton().onTrue(intake.runMotor());
      getStopMotorButton().onTrue(intake.stopMotor());
    }

  }

  @Override
  public void registerCommands() {
    registerIntake();
  }
}
