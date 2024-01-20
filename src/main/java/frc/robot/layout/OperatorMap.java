package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Positionmotor;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getPositionmotorButton();

  private void registerIntake() {}

  @Override
  public void registerCommands() {
    registerIntake();

    var pMotor = Positionmotor.getInstance();
    getPositionmotorButton().onTrue(pMotor.runMotorToSetpoint());
  }
}
