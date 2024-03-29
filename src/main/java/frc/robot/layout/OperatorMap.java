package frc.robot.layout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.PoseEstimator;
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

    abstract JoystickButton getChangeClimberLockStatusButton();

    abstract double getManualPivotAxis();

    abstract double getManualClimberAxis();

    abstract JoystickButton getSpeakerShotAlignButton();

    abstract JoystickButton getFerryShotAlignButton();

    abstract JoystickButton getTrapButton();

    abstract JoystickButton getStageAlignButton();

    abstract JoystickButton getManualShootButton();

    abstract JoystickButton getEjectButton();

    abstract JoystickButton getAmplifyButton();

    abstract JoystickButton getCoopButton();

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
            // getOuttakeButton().onTrue(new InstantCommand(() ->
            // intake.setIntakeState(Intake.IntakeDirection.REVERSE), intake));

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

            getChangeClimberLockStatusButton().onTrue(climber.toggleClimberLockStatusCommand());
        }
    }

    private void registerComplexCommands() {
        if (Config.Subsystems.INTAKE_ENABLED && Config.Subsystems.FEEDER_ENABLED) {
            getIntakeButton().onTrue(new IntakeUntilLoadedCommand());
        }

        if (Config.Subsystems.CLIMBER_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
            var pivot = Pivot.getInstance();
            var climber = Climber.getInstance();
            
            getClimbSequenceButton()
                    .whileTrue(pivot.setPositionCommand(() -> PivotMap.PIVOT_AMP_ANGLE).andThen(climber.run(() -> 0.3)));
            getClimbSequenceButton().onFalse(climber.run(() -> -0.3));
        }

        if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.PIVOT_ENABLED) {

            Shooter shooter = Shooter.getInstance();
            Pivot pivot = Pivot.getInstance();
            getSubwooferShotButton().onTrue(pivot.setPositionCommand(() -> ActionSetpoint.SUBWOOFER_SHOT.getAngle()));
            getSubwooferShotButton()
                    .onTrue(shooter.setFlywheelVelocityCommand(() -> ActionSetpoint.SUBWOOFER_SHOT.getRPM()));
            getPodiumShotButton().onTrue(pivot.setPositionCommand(() -> ActionSetpoint.PODIUM_SHOT.getAngle()));
            getPodiumShotButton().onTrue(shooter.setFlywheelVelocityCommand(() -> ActionSetpoint.PODIUM_SHOT.getRPM()));
            getSubwooferShotButton().onFalse(pivot.setPositionCommand(() -> 0.0));
            getSubwooferShotButton().onFalse(shooter.stopFlywheelCommand());
            getPodiumShotButton().onFalse(pivot.setPositionCommand(() -> 0.0));
            getPodiumShotButton().onFalse(shooter.stopFlywheelCommand());

        }

        if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.INTAKE_ENABLED
                && Config.Subsystems.DRIVETRAIN_ENABLED && Config.Subsystems.PIVOT_ENABLED) {
            Shooter shooter = Shooter.getInstance();
            Pivot pivot = Pivot.getInstance();
            FlywheelLookupTable speakerLookupTable = ShooterMap.SPEAKER_LOOKUP_TABLE;
            FlywheelLookupTable ferryLookupTable = ShooterMap.FERRY_LOOKUP_TABLE;

            // if alliance isn't populating default to red to avoid null pointer
            Supplier<Pose2d> speakerTarget = () -> DriverStation.getAlliance().isPresent()
                    && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER
                            : Coordinates.RED_SPEAKER;

            Supplier<Pose2d> ferryTarget = () -> new Pose2d(0,0, new Rotation2d(0)); //TODO: Find Target Pose

            PoseEstimator poseEstimator = PoseEstimator.getInstance();

            // getShootSpeakerButton().onTrue(new ShootSequenceCommand());

            Supplier<ActionSetpoint> getSpeakerActionSetpoint = () -> speakerLookupTable
                    .get(poseEstimator.getDistanceToPose(speakerTarget.get().getTranslation()));

            getSpeakerShotAlignButton().whileTrue(
                    pivot.setPositionCommand(() -> getSpeakerActionSetpoint.get().getAngle())
                            .alongWith(
                                    shooter.setFlywheelVelocityCommand(() -> getSpeakerActionSetpoint.get().getRPM())));

            getSpeakerShotAlignButton()
                    .onFalse(shooter.stopFlywheelCommand().alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));


            Supplier<ActionSetpoint> getFerryActionSetpoint = () -> ferryLookupTable.get(poseEstimator.getDistanceToPose(ferryTarget.get().getTranslation()));
            
            getFerryShotAlignButton().whileTrue(
                    pivot.setPositionCommand(() -> getFerryActionSetpoint.get().getAngle())
                            .alongWith(
                                    shooter.setFlywheelVelocityCommand(() -> getFerryActionSetpoint.get().getRPM())
                            ));

            getFerryShotAlignButton().onFalse(shooter.stopFlywheelCommand().alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));

            getAmpAlignButton().onTrue(
                    pivot.setPositionCommand(() -> PivotMap.PIVOT_AMP_ANGLE)
                            .alongWith(shooter.setFlywheelVelocityCommand(() -> ShooterMap.AMP_SPEED))
                            );

            getAmpAlignButton()
                    .onFalse(shooter.stopFlywheelCommand().alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));

            getStageAlignButton().onTrue(pivot.setPositionCommand(() -> PivotMap.PIVOT_TRAP_ANGLE)
                    .alongWith(shooter.setFlywheelVelocityCommand(() -> ShooterMap.TRAP_SPEED)));
            getStageAlignButton()
                    .onFalse(shooter.stopFlywheelCommand().alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));
                


            // getEjectButton().whileTrue(new InstantCommand(() ->
            // feeder.setFeederState(FeederDirection.REVERSE)).alongWith(intake.se));

            // TODO: Make a button mapped to this
            var shooterIntakeCommand = new ParallelCommandGroup(
                new InstantCommand(() -> Feeder.getInstance().setFeederState(FeederDirection.REVERSE)),
                shooter.setFlywheelVelocityCommand(() -> ShooterMap.SHOOTER_INTAKE_SPEED),
                    pivot.setPositionCommand(() -> PivotMap.PIVOT_INTAKE_ANGLE)
            ).until(() -> Feeder.getInstance().isNoteLoaded() && !Feeder.getInstance().getUpperBeamBreak())
                    .andThen(new InstantCommand(() -> Feeder.getInstance().setFeederState(FeederDirection.STOPPED)))
                    .andThen(shooter.stopFlywheelCommand())
                    .andThen(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
        }

    }

    private void registerPivot() {
        if (Config.Subsystems.PIVOT_ENABLED) {
            Pivot pivot = Pivot.getInstance();

            getPivotRaiseButton().onTrue(pivot.setPositionCommand(() -> (PivotMap.PIVOT_AMP_ANGLE)));
            getPivotLowerButton().onTrue(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
        }
    }

    private void registerLEDs() {
        AddressableLEDLights lights = AddressableLEDLights.getInstance();
        Feeder feeder = Feeder.getInstance();

        getAmplifyButton().onTrue(
                lights.getAmplifyPatternCommand()
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withTimeout(4.0)
        // .andThen(lights.setNoteStatusCommand(getAmpAlignButton()::getAsBoolean)
        );

        getCoopButton().onTrue(
                lights.getCoOpPatternCommand()
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                        .withTimeout(4.0)
        // .andThen(lights.setNoteStatusCommand(getAmpAlignButton()::getAsBoolean))
        );

        getIntakeButton().whileTrue(lights.setAlingmentNoteStatusCommand(feeder::isNoteLoaded).repeatedly());

        // will get canceled on both triggers until the rising edge is detected
        // lights.setDefaultCommand(lights.getCoOpPatternCommand());
        lights.setDefaultCommand(lights.setNoteStatusCommand(feeder::isNoteLoaded));
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
        registerPivot();
        registerLEDs();
        registerComplexCommands();

        // registerSubsystems();
    }
}
