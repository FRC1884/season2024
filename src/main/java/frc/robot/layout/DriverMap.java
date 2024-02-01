package frc.robot.layout;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ExampleConfig;
import frc.robot.RobotMap;
import frc.robot.RobotMap.PrototypeMap;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Prototypes;
import frc.robot.subsystems.test;
import frc.robot.subsystems.Vision.PoseEstimator;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract double getSwerveXSpeed();

  abstract double getSwerveYSpeed();

  abstract double getSwerveRot();

  abstract JoystickButton getArcingButton();

  abstract JoystickButton getTestButton();

  abstract JoystickButton getFollowAprilTagButton();

  abstract JoystickButton getFollowNoteButton();

  abstract JoystickButton getResetOdometryVisionButton();

  private void registerDrivetrain() {
    if (ExampleConfig.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = Drivetrain.getInstance();
      // drivetrain.setDefaultCommand(
      // drivetrain.driveCommand(
      // this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));
      // getTestButton().onTrue(drivetrain.followPathCommand("ShortTestPath", true));
      getArcingButton().whileFalse(drivetrain.driveCommand(
          this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));
      getArcingButton().whileTrue(drivetrain.driveSetAngleCommand(
              this::getSwerveXSpeed,this::getSwerveYSpeed));
      getTestButton()
          .onTrue(drivetrain.TestAllCommand());
      getFollowAprilTagButton().whileTrue(drivetrain.followAprilTagCommand());
      getFollowNoteButton().onTrue(drivetrain.followNoteCommand());
      getResetOdometryVisionButton().onTrue(PoseEstimator.getInstance().resetOdometryVisionCommand());
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
