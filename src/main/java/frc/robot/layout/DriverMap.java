package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.ExampleConfig;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract JoystickButton getFollowAprilTagButton();

  private void registerDrivetrain() {
    if (ExampleConfig.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = Drivetrain.getInstance();
      drivetrain.setDefaultCommand(drivetrain.driveCommand(this::getChassisSpeeds));
      getFollowAprilTagButton().whileTrue(drivetrain.followAprilTagCommand());
    }

  }

  @Override
  public void registerCommands() {
    if (ExampleConfig.Subsystems.DRIVETRAIN_ENABLED)
      registerDrivetrain();
  }
}
