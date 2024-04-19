package frc.robot.layout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.util.ActionSetpoint;
import frc.robot.util.FlywheelLookupTable;

import java.util.function.Supplier;
import java.util.function.BooleanSupplier;

public abstract class OperatorMap extends CommandMap {

    public OperatorMap(GameController controller) {
        super(controller);
    }

    abstract Trigger getIntakeButton();

    abstract Trigger getOuttakeButton();

    abstract Trigger getSourceIntakeButton();

    abstract Trigger getShootSpeakerButton();

    abstract Trigger getShootAmpButton();

    abstract Trigger getAmpAlignButton();

    abstract Trigger getClimbSequenceButton();

    abstract Trigger getBackPodiumShotButton();

    abstract double getManualPivotAxis();

    abstract double getManualClimberAxis();

    abstract Trigger getSpeakerShotAlignButton();

    abstract Trigger getFerryShotAlignButton();

    abstract Trigger getTrapButton();

    abstract Trigger getStageAlignButton();

    abstract Trigger getEjectButton();

    abstract Trigger getAmplifyButton();

    abstract Trigger getCoopButton();

    abstract Trigger getLEDPatternOffButton();

    abstract Trigger getPivotRaiseButton();

    abstract Trigger getPivotLowerButton();

    abstract double getLEDAxis1();

    abstract double getLEDAxis2();

    abstract Trigger getClimberRaiseButton();

    abstract Trigger getClimberLowerButton();

    abstract Trigger getSubwooferShotButton();

    abstract Trigger getPodiumShotButton();

    private void registerIntake() {
        if (Config.Subsystems.INTAKE_ENABLED) {
            Intake intake = Intake.getInstance();
            getOuttakeButton()
                    .onTrue(new InstantCommand(() -> intake.setIntakeState(Intake.IntakeDirection.REVERSE), intake));

        }
    }

    private void registerShooter() {
        if (Config.Subsystems.SHOOTER_ENABLED) {
        }
    }

    private void registerFeeder() {
        Feeder feeder = Feeder.getInstance();
        Pivot pivot = Pivot.getInstance();

        var feederForward = new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD));
        var feederReverse = new InstantCommand(() -> feeder.setFeederState(FeederDirection.REVERSE));
        var feederStop = new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED));
        var feederSlow = new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD_SLOW));
        BooleanSupplier pivotReady = pivot::isAtGoal;

        // if (pivotReady.getAsBoolean()){ //ONLY USE DURING PRACTICE MATCHES!!!
                getShootSpeakerButton().whileTrue(feederForward);
                getShootSpeakerButton().onFalse(feederStop);
        // }


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
            Pivot pivot = Pivot.getInstance();
            // climber.setDefaultCommand(climber.run(this::getManualClimberAxis));

            getClimberRaiseButton().whileTrue(Climber.getInstance().run(() -> 0.9)
                    .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));
            getClimberRaiseButton().onFalse(Climber.getInstance().run(() -> -0.0));
            getClimberLowerButton().whileTrue(Climber.getInstance().run(() -> -0.9)
                    .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));
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

            getClimbSequenceButton()
                    .whileTrue(
                            pivot.setPositionCommand(() -> PivotMap.PIVOT_AMP_ANGLE).andThen(climber.run(() -> 0.3)));
            getClimbSequenceButton().onFalse(climber.run(() -> -0.3));
        }

        if (Config.Subsystems.SHOOTER_ENABLED && Config.Subsystems.PIVOT_ENABLED) {

            Shooter shooter = Shooter.getInstance();
            Pivot pivot = Pivot.getInstance();
            getSubwooferShotButton().whileTrue(pivot.setPositionCommand(() -> ShooterMap.FERRY_SETPOINT_3500_06.getAngle()));
            getSubwooferShotButton().whileTrue(shooter.setFlywheelVelocityCommand(() -> ShooterMap.FERRY_SETPOINT_3500_06.getRPM()));

            getSubwooferShotButton().onFalse(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
            getSubwooferShotButton().onFalse(shooter.stopFlywheelCommand());

            getPodiumShotButton().whileTrue(pivot.setPositionCommand(() -> ShooterMap.FERRY_SETPOINT_3500_07.getAngle()));
            getPodiumShotButton().whileTrue(shooter.setFlywheelVelocityCommand(() -> ShooterMap.FERRY_SETPOINT_3500_07.getRPM()));

            getPodiumShotButton().onFalse(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
            getPodiumShotButton().onFalse(shooter.stopFlywheelCommand());

            getBackPodiumShotButton().whileTrue(pivot.setPositionCommand(() -> ShooterMap.BACK_PODIUM_SETPOINT.getAngle()));
            getBackPodiumShotButton().whileTrue(shooter.setFlywheelVelocityCommand(() -> ShooterMap.BACK_PODIUM_SETPOINT.getRPM()));

            getBackPodiumShotButton().onFalse(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
            getBackPodiumShotButton().onFalse(shooter.stopFlywheelCommand());

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

        //     Supplier<Pose2d> ferryTarget = () -> DriverStation.getAlliance().isPresent()
        //     && (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? Coordinates.BLUE_FERRY
        //             : Coordinates.RED_FERRY;

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

        //     Supplier<ActionSetpoint> getFerryActionSetpoint = () -> ferryLookupTable
        //             .get(poseEstimator.getDistanceToPose(ferryTarget.get().getTranslation()));

        //     getFerryShotAlignButton()
        //             .onTrue(new InstantCommand(() -> System.out.println(getFerryActionSetpoint.get().getRPM())))
        //             .whileTrue(
        //                     pivot.setPositionCommand(() -> getFerryActionSetpoint.get().getAngle())
        //                             .alongWith(
        //                                     shooter.setFlywheelVelocityCommand(
        //                                             () -> getFerryActionSetpoint.get().getRPM())));

        //     getFerryShotAlignButton().onFalse(shooter.stopFlywheelCommand()
        //             .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));

        getFerryShotAlignButton().whileTrue(pivot.setPositionCommand(() -> ShooterMap.FERRY_SETPOINT.getAngle()));
        getFerryShotAlignButton().whileTrue(shooter.setFlywheelVelocityCommand(() -> ShooterMap.FERRY_SETPOINT.getRPM()));

        getFerryShotAlignButton().onFalse(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE));
        getFerryShotAlignButton().onFalse(shooter.stopFlywheelCommand());

            getAmpAlignButton().onTrue(
                    pivot.setPositionCommand(() -> PivotMap.PIVOT_AMP_ANGLE)
                            .alongWith(shooter.setFlywheelVelocityIndividuallyCommand(() -> ShooterMap.AMP_SPEED_LEAD,
                                    () -> ShooterMap.AMP_SPEED_FOLLOW)));

            getAmpAlignButton()
                    .onFalse(shooter.stopFlywheelCommand()
                            .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));

            getStageAlignButton().onTrue(pivot.setPositionCommand(() -> PivotMap.PIVOT_TRAP_ANGLE)
                    .alongWith(shooter.setFlywheelVelocityCommand(() -> ShooterMap.TRAP_SPEED)));
            getStageAlignButton()
                    .onFalse(shooter.stopFlywheelCommand()
                            .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));

            // getEjectButton().whileTrue(new InstantCommand(() ->
            // feeder.setFeederState(FeederDirection.REVERSE)).alongWith(inta));

            // TODO: Make a button mapped to this
            var raiseShooter = new InstantCommand(() -> Feeder.getInstance().setFeederState(FeederDirection.REVERSE))
                    .alongWith(
                            shooter.setFlywheelVelocityCommand(() -> ShooterMap.SHOOTER_INTAKE_SPEED))
                    .alongWith(
                            pivot.setPositionCommand(() -> PivotMap.PIVOT_INTAKE_ANGLE));

            getSourceIntakeButton()
                    .onTrue(raiseShooter.until(
                            () -> Feeder.getInstance().isNoteLoaded() && Feeder.getInstance().getUpperBeamBreak())
                            .andThen(new ParallelCommandGroup(
                                    new InstantCommand(
                                            () -> Feeder.getInstance().setFeederState(FeederDirection.STOPPED)),
                                    shooter.stopFlywheelCommand(),
                                    pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE))))
                    .onFalse(new InstantCommand(() -> Feeder.getInstance().setFeederState(FeederDirection.STOPPED))
                            .alongWith(shooter.stopFlywheelCommand())
                            .alongWith(pivot.setPositionCommand(() -> PivotMap.PIVOT_RESTING_ANGLE)));
        }

    }


    //currently doesn't do anything cuz not registered cuz pivot gonna kill bot when climbing
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

        getSubwooferShotButton().whileTrue(Commands.either(
            lights.setBlinkingCommand(Color.kIndigo, Color.kBlue, 5.0), 
            lights.setAlingmentNoteStatusCommand(() -> !Feeder.getInstance().isNoteLoaded()), 
            () -> {
//                System.out.println(Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity());
                return Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity();
            }
            ).repeatedly());

        getPodiumShotButton().whileTrue(Commands.either(
                lights.setBlinkingCommand(Color.kIndigo, Color.kBlue, 5.0),
                lights.setAlingmentNoteStatusCommand(() -> !Feeder.getInstance().isNoteLoaded()),
                () -> {
//                System.out.println(Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity());
                    return Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity();
                }
        ).repeatedly());

        getSpeakerShotAlignButton().whileTrue(Commands.either(
                lights.setBlinkingCommand(Color.kIndigo, Color.kBlue, 5.0),
                lights.setAlingmentNoteStatusCommand(() -> !Feeder.getInstance().isNoteLoaded()),
                () -> {
//                System.out.println(Pivot.getInstance().isAtGoal() && Shooter.getInstance().getFlywheelIsAtVelocity());
                    return Shooter.getInstance().isReadyToShoot();
                }
        ).repeatedly());

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

        getIntakeButton().whileTrue(lights.setAlingmentNoteStatusCommand(() -> !feeder.isNoteLoaded()).repeatedly());

        // getClimberLowerButton().whileTrue(lights.setGBFlagCommand());
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
        //TODO: Remove due to climber possibly running when moving joystick
        //registerPivot(); 
        registerLEDs();
        registerComplexCommands();

        // registerSubsystems();
    }
}
