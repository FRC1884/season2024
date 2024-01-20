package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.kitdrivetrain;
import frc.robot.subsystems.test;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract double getKitDrivetrainRot();

  abstract double getKitDrivetrainForward();

  private void registerDrivetrain() {
    if (ExampleConfig.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = kitdrivetrain.getInstance();
      drivetrain.setDefaultCommand(
          drivetrain.driveCommand(
              this::getKitDrivetrainForward, this::getKitDrivetrainRot));

      // getFollowAprilTagButton().whileTrue(drivetrain.followAprilTagCommand());
      // getSpeakerOrSourceButton()
      // .onTrue(
      // drivetrain.goSpeakerOrSource(
      // false)); // boolean arguement set as false as function to determine if robot
      // is
      // holding note has not been created yet
    }
  }

  @Override
  public void registerCommands() {
    registerDrivetrain();
  }
}
