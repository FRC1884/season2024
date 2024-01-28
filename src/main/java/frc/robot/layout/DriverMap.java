package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Config;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }


  abstract double getSwerveXSpeed();

  abstract double getSwerveYSpeed();

  abstract double getSwerveRot();

  abstract JoystickButton getTestButton();

  abstract JoystickButton getFollowAprilTagButton();

  abstract JoystickButton getSourceToSpeakerButton();

  abstract JoystickButton getSourceToAmpButton();

  abstract JoystickButton getSpeakerToSourceButton();

  abstract JoystickButton getSpeakerToStageButton();

  abstract JoystickButton getSpeakerOrSourceButton();

  private void registerDrivetrain() {
    if (Config.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = Drivetrain.getInstance();
      drivetrain.setDefaultCommand(
          drivetrain.driveCommand(
              this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));
      getTestButton().onTrue(drivetrain.followPathCommand("ShortTestPath", true));
      getFollowAprilTagButton().whileTrue(drivetrain.followAprilTagCommand());
    }
  }

  @Override
  public void registerCommands() {
    registerDrivetrain();
    getSpeakerOrSourceButton().onTrue(new PrintCommand("Hello"));
  }
}
