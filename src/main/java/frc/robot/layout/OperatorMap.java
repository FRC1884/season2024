package frc.robot.layout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PivotMap;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.util.FlywheelLookupTable;

public abstract class OperatorMap extends CommandMap {

  public OperatorMap(GameController controller) {
    super(controller);
  }

  abstract JoystickButton getIntakeStopButton();

  abstract JoystickButton getIntakeButton();

  abstract JoystickButton getIntakeReverseButton();

  abstract JoystickButton getShootButton();

  abstract JoystickButton getShootAmpButton();

  abstract JoystickButton getAmpAlignButton();

  abstract JoystickButton getClimbSequenceButton();

  abstract double getManualPivotAxis();

  abstract JoystickButton getArcButton();

  abstract JoystickButton getTrapButton();

  abstract JoystickButton getStageAlignButton();

  abstract JoystickButton getManualShootButton();

  abstract JoystickButton getAmplifyButton();

  abstract JoystickButton getCoopButton();

  abstract JoystickButton getLEDPatternOneButton();

  abstract JoystickButton getLEDPatternTwoButton();

  abstract JoystickButton getLEDPatternThreeButton();

  abstract JoystickButton getLEDPatternFourButton();

  abstract JoystickButton getLEDPatternFiveButton();

  abstract JoystickButton getLEDPatternOffButton();

  abstract Trigger getPivotLowerButton();

  abstract Trigger getPivotRaiseButton();

  abstract double getLEDAxis1();

  abstract double getLEDAxis2();

  private void registerIntake() {
    if (Config.Subsystems.INTAKE_ENABLED) {
      Intake intake = Intake.getInstance();
      // getIntakeButton().onTrue(intake.setIntakeState(Intake.IntakeDirection.FORWARD).andThen(
      // new InstantCommand(() -> shooter.setFeederState(FeederDirection.FORWARD))
      // ).until(() -> shooter.isNoteLoaded()).andThen(
      // intake.setIntakeState(Intake.IntakeDirection.STOPPED)
      // ));

      getIntakeStopButton().onTrue(intake.setIntakeState(IntakeDirection.STOPPED));
      getIntakeButton().whileTrue(intake.intakeUntilLoadedCommand())
              .onFalse(intake.setIntakeState(IntakeDirection.STOPPED));
      getIntakeReverseButton().onTrue(intake.setIntakeState(IntakeDirection.REVERSE));


      getShootButton().onTrue(intake.setIntakeState(IntakeDirection.FORWARD))
              .onFalse(intake.setIntakeState(IntakeDirection.STOPPED));
    }
  }

  private void registerShamper() {
    if (Config.Subsystems.SHAMPER_ENABLED) {
      Shamper shooter = Shamper.getInstance();
      Pivot pivot = Pivot.getInstance();
      FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
      Pose2d target = DriverStation.getAlliance().equals(DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER
              : Coordinates.RED_SPEAKER;
      PoseEstimator poseEstimator = PoseEstimator.getInstance();
    }
  }

  // private void registerFeeder() {
  // if(Config.Subsystems.FEEDER_ENABLED) {
  // Feeder feeder = Feeder.getInstance();
  // getShootSpeakerButton().whileTrue(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.FORWARD)));
  // getShootAmpButton().whileTrue(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.FORWARD_SLOW)));
  // getTrapButton().whileTrue(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.FORWARD)));
  // getShootSpeakerButton().onFalse(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.STOPPED)));
  // getShootAmpButton().onFalse(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.STOPPED)));
  // getTrapButton().onFalse(new InstantCommand(() ->
  // feeder.setFeederState(FeederDirection.STOPPED)));
  // }
  // }

  private void registerClimber() {
    if (Config.Subsystems.CLIMBER_ENABLED) {
      Climber climber = Climber.getInstance();

    }
  }

  private void registerComplexCommands() {
    if (Config.Subsystems.SHAMPER_ENABLED && Config.Subsystems.INTAKE_ENABLED
            && Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.CLIMBER_ENABLED) {
      Intake intake = Intake.getInstance();
      Shamper shooter = Shamper.getInstance();
      Pivot pivot = Pivot.getInstance();
      Climber climber = Climber.getInstance();
      FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
      Pose2d target = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER
              : Coordinates.RED_SPEAKER;
      PoseEstimator poseEstimator = PoseEstimator.getInstance();
      // getShootSpeakerButton().onTrue(new ShootSequenceCommand());

      getArcButton().whileTrue((pivot.updatePosition(() -> lookupTable
                      .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint())
              .alongWith(shooter.setShootVelocityCommand(() -> lookupTable.get(
                              poseEstimator.getDistanceToPose(target.getTranslation())).getFlywheelV(),
                      () -> lookupTable.get(poseEstimator.getDistanceToPose(target.getTranslation())).getFeederV()))));

      getArcButton().onFalse(shooter.setShootVelocityCommand(() -> 0.0, () -> 0.0));

      getShootAmpButton().onTrue(intake.setIntakeState(IntakeDirection.FORWARD).andThen(shooter.setFeederVelocityCommand(() -> ShamperMap.AMP_SPEED_FEEDER)))
              .onFalse(intake.setIntakeState(IntakeDirection.STOPPED));

      getAmpAlignButton().onTrue(

              pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).alongWith(
                      shooter.setTopVelocityCommand(() -> ShamperMap.AMP_SPEED_TOP).andThen(shooter.setBotVelocityCommand(() -> -ShamperMap.AMP_SPEED_TOP))));

      getAmpAlignButton().onFalse(
              pivot.updatePosition(() -> PivotMap.LOWER_SETPOINT_LIMIT + 1).alongWith(
                      shooter.setFlywheelVelocityCommand(() -> 0.0).andThen(shooter.setFeederVelocityCommand(() -> 0.0))));

      getStageAlignButton().onTrue(
              pivot.updatePosition(() -> PivotMap.PIVOT_TRAP_ANGLE).alongWith(
                      shooter.setFlywheelVelocityCommand(() -> ShamperMap.TRAP_SPEED)));

      getClimbSequenceButton()
        // keep it held for 0.5s for it to take effect (up to operator preference)
        .debounce(0.5)
        .onTrue(
        Commands.repeatingSequence(
          // raise the climber and move the pivot out of the way at the same time
          Climber.getInstance().updatePosition(() -> 2.0).alongWith(
            pivot.updatePosition(() -> 0.0)
          )
            // both of the updates will run constantly, so cancel both
            // of them after a reasonable amount of time has passed
            // so that they don't overtake the rest of the sequence
            .withTimeout(2.0),
          // pretty sure having drive in here is non-negotiable
          // since we need to align
          // drive up to an offset from the tag
          // can i actually do this? will the z-axis make a genuine difference?
          Drivetrain.getInstance().alignToTagCommand(new Pose2d(1.0, 0.0, new Rotation2d())),
          // if not, just pathplan a forward distance:
          // Drivetrain.getInstance().goToPoint(
            // Drivetrain.getInstance().getPose().plus(new Transform2d(1.0, 0.0, new Rotation2d()))),
          Commands.waitSeconds(1.0),
          // lower elevator -> climb!
          Climber.getInstance().updatePosition(() -> 0.0)
        )
      );

      getPivotLowerButton().onTrue(pivot.updatePosition(() -> 0.0).alongWith(new PrintCommand("hi")));
      getPivotRaiseButton().onTrue(pivot.updatePosition(() -> 75.0).alongWith(new PrintCommand("bye")));
    }

  }

  private void registerLEDs() {
    if (Config.Subsystems.LEDS_ENABLED && Config.Subsystems.INTAKE_ENABLED) {
      AddressableLEDLights lights = AddressableLEDLights.getInstance();
      Intake intake = Intake.getInstance();

      getAmplifyButton().onTrue(lights.toggleAmplifyState(intake::getNoteStatus));
      getCoopButton().onTrue(lights.toggleCoopState(intake::getNoteStatus));
      lights.setDefaultCommand(lights.useState(lights::getState));
    }
  }

  @Override
  public void registerCommands() {
    // registerPrototype();
    registerIntake();
    registerClimber();
    registerShamper();
    //registerLEDs();
    registerComplexCommands();
  }
}