package frc.robot.layout;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.function.DoubleSupplier;

import javax.print.attribute.standard.PrinterMessageFromOperator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.core.util.controllers.ButtonMap.Axis;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PivotMap;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.core.util.controllers.BoardController;
import frc.robot.util.BlinkinUtils;
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

  abstract Trigger getPivotRaiseButton();

  abstract Trigger getPivotLowerButton();

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

      getShootAmpButton().onTrue(intake.setIntakeState(IntakeDirection.FORWARD))
          .onFalse(intake.setIntakeState(IntakeDirection.STOPPED));
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
        && Config.Subsystems.DRIVETRAIN_ENABLED) {
      Intake intake = Intake.getInstance();
      Shamper shooter = Shamper.getInstance();
      Pivot pivot = Pivot.getInstance();
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

      getAmpAlignButton().onTrue(
        
          pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).alongWith(
              shooter.setTopVelocityCommand(() -> ShamperMap.AMP_SPEED_TOP).andThen(shooter.setBotVelocityCommand(() -> -ShamperMap.AMP_SPEED_TOP)).andThen(shooter.setFeederVelocityCommand(() -> ShamperMap.AMP_SPEED_FEEDER))));

      getAmpAlignButton().onFalse(
          pivot.updatePosition(() -> 0.0).alongWith(
              shooter.setFlywheelVelocityCommand(() -> 0.0).andThen(shooter.setFeederVelocityCommand(() -> 0.0))));

      getStageAlignButton().onTrue(
          pivot.updatePosition(() -> PivotMap.PIVOT_TRAP_ANGLE).alongWith(
              shooter.setFlywheelVelocityCommand(() -> ShamperMap.TRAP_SPEED)));

      getClimbSequenceButton().onTrue(
          new SequentialCommandGroup(
              Climber.getInstance().run(() -> 0.2),
              new WaitCommand(1),
              Climber.getInstance().run(() -> 0)));
      getPivotRaiseButton().onTrue(pivot.updatePosition(() -> -115.0));
      getPivotLowerButton().onTrue(pivot.updatePosition(() -> -1.0));
    }

  }

  private void registerLEDs() {
    if (Config.Subsystems.LEDS_ENABLED) {
      AddressableLEDLights lights = AddressableLEDLights.getInstance();
      // getLEDPatternOffButton().whileTrue(lights.disableCommand());
      // // getLEDPatternOneButton().onTrue(lights.setRedBlack());
      // getLEDPatternTwoButton().whileTrue(lights.setDecreasing());
      // getLEDPatternThreeButton().whileTrue(lights.setRainbow());
      // getLEDPatternFourButton().whileTrue(lights.setRedDarkRed());
      // getLEDPatternFiveButton().whileTrue(lights.checkBeam());
      lights.setDefaultCommand(lights.setValue(this::getLEDAxis1, this::getLEDAxis2));
    }
  }

  @Override
  public void registerCommands() {
    // registerPrototype();
    registerIntake();
    registerClimber();
    registerShamper();
    registerLEDs();
    registerComplexCommands();
  }
}
