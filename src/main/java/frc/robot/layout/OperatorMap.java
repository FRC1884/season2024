package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Shooter;
import frc.robot.RobotMap.ShooterMap;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getTestButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getFlywheelButton();

  private void registerIntake() {}

  @Override
  public void registerCommands() {
    registerIntake();
    var pMotor = Shooter.getInstance(); 
    getIntakeButton().onTrue(pMotor.runShooter());

    getFlywheelButton().whileTrue(
      pMotor.prepareShootCommand()
          .withTimeout(ShooterMap.shootDelay)
          .andThen(pMotor.launchNoteCommand())
          .handleInterrupt(pMotor::stopMotors));
    
  }
}
