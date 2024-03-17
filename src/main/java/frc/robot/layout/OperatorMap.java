package frc.robot.layout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.core.util.controllers.CommandMap;
import frc.robot.core.util.controllers.GameController;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.Config;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.PivotMap;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.util.ActionSetpoint;
import frc.robot.util.FlywheelLookupTable;

import java.util.function.Supplier;

public abstract class OperatorMap extends CommandMap {

    public OperatorMap(GameController controller) {
        super(controller);
    }

    abstract JoystickButton getIntakeButton();

    abstract JoystickButton getOuttakeButton();

    abstract JoystickButton getShootSpeakerButton();

    abstract JoystickButton getShootAmpButton();

    abstract JoystickButton getAmpAlignButton();

    abstract JoystickButton getClimbSequenceButton();

    abstract double getManualPivotAxis();

    abstract double getManualClimberAxis();

    abstract JoystickButton getArcButton();

    abstract JoystickButton getTrapButton();

    abstract JoystickButton getStageAlignButton();

    abstract JoystickButton getManualShootButton();

    abstract JoystickButton getEjectButton();

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

    abstract Trigger getClimberRaiseButton();

    abstract Trigger getClimberLowerButton();

    abstract JoystickButton getSubwooferShotButton();

    abstract JoystickButton getPodiumShotButton();

    private void registerIntake() {
        if (Config.Subsystems.INTAKE_ENABLED) {
            Intake intake = Intake.getInstance();
            // getOuttakeButton().onTrue(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeDirection.REVERSE), intake));


        }
    }

    private void registerShooter() {
        if (Config.Subsystems.SHOOTER_ENABLED) {
        }
    }

    private void registerFeeder() {
        Feeder feeder = Feeder.getInstance();

        var feederForward = new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD));
        var feederReverse = new InstantCommand(() -> feeder.setFeederState(FeederDirection.REVERSE));
        var feederStop = new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED));
        var feederSlow = new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD_SLOW));

        getShootSpeakerButton().whileTrue(feederForward);
        getShootSpeakerButton().onFalse(feederStop);

        getShootAmpButton().whileTrue(feederSlow);
        getShootAmpButton().onFalse(feederStop);

        getTrapButton().whileTrue(feederForward);
        getTrapButton().onFalse(feederStop);

        getEjectButton().whileTrue(feederReverse);
        getEjectButton().onFalse(feederStop);
    }

    private void registerClimber() {
        if (Config.Subsystems.CLIMBER_ENABLED) {
            Climber climber = Climber.getInstance();
            climber.setDefaultCommand(climber.run(this::getManualClimberAxis));

            getClimberRaiseButton().whileTrue(Climber.getInstance().run(() -> 0.3));
            getClimberRaiseButton().onFalse(Climber.getInstance().run(() -> -0.0));
            getClimberLowerButton().whileTrue(Climber.getInstance().run(() -> -0.3));
            getClimberLowerButton().onFalse(Climber.getInstance().run(() -> -0.0));
        }
    }

    private void registerComplexCommands() {
        if (Config.Subsystems.INTAKE_ENABLED && Config.Subsystems.FEEDER_ENABLED) {
            getIntakeButton().whileTrue(new IntakeUntilLoadedCommand());
        }

        if (Config.Subsystems.CLIMBER_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
            var pivot = Pivot.getInstance();
            var climber = Climber.getInstance();

            getClimbSequenceButton().whileTrue(pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE).andThen(climber.run(() -> 0.3)));
            getClimbSequenceButton().onFalse(climber.run(() -> -0.3));
        }

        if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
            
            Shooter shooter = Shooter.getInstance();
            Pivot pivot = Pivot.getInstance();
            getSubwooferShotButton().onTrue(pivot.updatePosition(() -> ActionSetpoint.SUBWOOFER_SHOT.getAngle()));
            getSubwooferShotButton().onTrue(shooter.updatePosition(() -> ActionSetpoint.SUBWOOFER_SHOT.getRPM()));
            getPodiumShotButton().onTrue(pivot.updatePosition(() -> ActionSetpoint.PODIUM_SHOT.getAngle()));
            getPodiumShotButton().onTrue(shooter.updatePosition(() -> ActionSetpoint.PODIUM_SHOT.getRPM()));
        }

        if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.INTAKE_ENABLED && Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
            Shooter shooter = Shooter.getInstance();
            Pivot pivot = Pivot.getInstance();
            FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();

            // if alliance isn't populating default to red to avoid null pointer
            Supplier<Pose2d> target = () -> DriverStation.getAlliance().isPresent() && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;

            PoseEstimator poseEstimator = PoseEstimator.getInstance();

            // getShootSpeakerButton().onTrue(new ShootSequenceCommand());

            Supplier<ActionSetpoint> getActionSetpoint = () -> lookupTable.get(poseEstimator.getDistanceToPose(target.get().getTranslation()));

            getArcButton().whileTrue(
                    pivot.updatePosition(() -> getActionSetpoint.get().getAngle())
                            .alongWith(
                                    shooter.setFlywheelVelocityCommand(() -> getActionSetpoint.get().getRPM())
                            ));

            getArcButton().onFalse(shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(pivot.updatePosition(() -> -1.0)));

            getAmpAlignButton().onTrue(
                    pivot.updatePosition(() -> PivotMap.PIVOT_AMP_ANGLE)
                            .alongWith(shooter.setFlywheelVelocityCommand(() -> ShooterMap.AMP_SPEED)));

            getAmpAlignButton().onFalse(shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(pivot.updatePosition(() -> -1.0)));

            getStageAlignButton().onTrue(pivot.updatePosition(() -> PivotMap.PIVOT_TRAP_ANGLE).alongWith(shooter.setFlywheelVelocityCommand(() -> ShooterMap.TRAP_SPEED)));
            getStageAlignButton().onFalse(shooter.setFlywheelVelocityCommand(() -> 0.0).alongWith(pivot.updatePosition(() -> -1.0)));

            // getEjectButton().whileTrue(new InstantCommand(() -> feeder.setFeederState(FeederDirection.REVERSE)).alongWith(intake.se));
        }

    }

    private void registerPivot() {
        if (Config.Subsystems.PIVOT_ENABLED) {
            Pivot pivot = Pivot.getInstance();

            getPivotRaiseButton().onTrue(pivot.updatePosition(() -> (PivotMap.PIVOT_AMP_ANGLE + 40.0)));
            getPivotLowerButton().onTrue(pivot.updatePosition(() -> -1.0));
        }
    }


    private void registerLEDs() {
        AddressableLEDLights lights = AddressableLEDLights.getInstance();
      Intake intake = Intake.getInstance();

      getAmplifyButton().onTrue(
        lights.getAmplifyPatternCommand()
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
          .withTimeout(4.0)
          //.andThen(lights.setNoteStatusCommand(getAmpAlignButton()::getAsBoolean)
      );

      getCoopButton().onTrue(
        lights.getCoOpPatternCommand()
          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
          .withTimeout(4.0)
          //.andThen(lights.setNoteStatusCommand(getAmpAlignButton()::getAsBoolean))
      );

      // will get canceled on both triggers until the rising edge is detected
      //lights.setDefaultCommand(lights.getCoOpPatternCommand());
      lights.setDefaultCommand(lights.setNoteStatusCommand(()-> getIntakeButton().getAsBoolean()));
    }


    public void registerSubsystems() {
        Intake.getInstance();
        Shooter.getInstance();
        Feeder.getInstance();
        Climber.getInstance();
    }

  @Override
  public void registerCommands() {
    registerIntake();
    registerFeeder();
    registerClimber();
    registerShooter();
    //registerLEDs();
    registerComplexCommands();

        // registerSubsystems();
    }
}
