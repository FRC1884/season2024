package frc.robot.layout;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ExampleConfig;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.test;

public abstract class DriverMap extends CommandMap {

  public DriverMap(GameController controller) {
    super(controller);
  }

  abstract ChassisSpeeds getChassisSpeeds();

  abstract double getSwerveXSpeed();

  abstract double getSwerveYSpeed();

  abstract double getSwerveRot();

  abstract Trigger getTestButton();

  abstract Trigger getFollowAprilTagButton();

  abstract Trigger getSourceToSpeakerButton();

  abstract Trigger getSourceToAmpButton();

  abstract Trigger getSpeakerToSourceButton();

  abstract Trigger getSpeakerToStageButton();

  abstract Trigger getSpeakerOrSourceButton();

  private void registerDrivetrain() {
    if (ExampleConfig.Subsystems.DRIVETRAIN_ENABLED) {
      var drivetrain = Drivetrain.getInstance();
      drivetrain.setDefaultCommand(
          drivetrain.driveCommand(
              this::getSwerveXSpeed, this::getSwerveYSpeed, this::getSwerveRot));
      getTestButton().onTrue(drivetrain.followPathCommand("ShortTestPath", true));
      getFollowAprilTagButton().whileTrue(drivetrain.followAprilTagCommand());
      // getSpeakerOrSourceButton()
      //     .onTrue(
      //         drivetrain.goSpeakerOrSource(
      //             false)); // boolean arguement set as false as function to determine if robot is
      // holding note has not been created yet
    }
  }

  private void registerTestSub() {
    var t = test.getInstance();
    t.setDefaultCommand(t.testCommand(() -> getSwerveXSpeed()));
  }

  @Override
  public void registerCommands() {
    registerDrivetrain();
    getSpeakerOrSourceButton().onTrue(new PrintCommand("Hello"));
  }
}
