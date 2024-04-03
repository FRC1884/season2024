package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.Commands.IntakeUntilLoadedCommand;
import frc.robot.Commands.ShouldSkipNoteLogicCommand;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.ShooterMap;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Feeder.FeederDirection;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.util.FlywheelLookupTable;
import frc.robot.subsystems.PoseEstimator;

import java.util.function.Supplier;

public class AutoCommands {
        public static void registerAutoCommands() {
                FlywheelLookupTable lookupTable = ShooterMap.SPEAKER_LOOKUP_TABLE;

                // default to red speaker to avoid null pointer exception
                Supplier<Pose2d> getTarget = () -> DriverStation.getAlliance().isPresent()
                                && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                                                ? Coordinates.BLUE_SPEAKER
                                                : Coordinates.RED_SPEAKER;

                PoseEstimator poseEstimator = PoseEstimator.getInstance();
                Pivot pivot = Pivot.getInstance();
                Shooter shooter = Shooter.getInstance();
                Feeder feeder = Feeder.getInstance();
                Drivetrain drivetrain = Drivetrain.getInstance();
                Vision vision = Vision.getInstance();
                Intake intake = Intake.getInstance();

                NamedCommands.registerCommand("Print", Commands.print("woo something happened"));

                NamedCommands.registerCommand("Intake", new IntakeUntilLoadedCommand());

                NamedCommands.registerCommand("Feed On", new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD),
                                                Feeder.getInstance()));

                NamedCommands.registerCommand("Feed Off", new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED),
                                                Feeder.getInstance()));

                NamedCommands.registerCommand("Intake On", new InstantCommand(() -> intake.setIntakeState(IntakeDirection.FORWARD),
                                                Intake.getInstance()));

                NamedCommands.registerCommand("Intake Off", new InstantCommand(() -> intake.setIntakeState(IntakeDirection.STOPPED),
                                                Intake.getInstance()));

                

                NamedCommands.registerCommand("SpoolShooter", shooter.setFlywheelVelocityCommand(
                                () -> lookupTable.get(poseEstimator.getDistanceToPose(getTarget.get().getTranslation()))
                                                .getRPM()));

                NamedCommands.registerCommand("Pivot", pivot.setPositionCommand(
                                () -> lookupTable.get(poseEstimator.getDistanceToPose(getTarget.get().getTranslation()))
                                                .getAngle()));

                NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                                new InstantCommand(() -> feeder.setFeederState(FeederDirection.FORWARD),
                                                Feeder.getInstance()),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> feeder.setFeederState(FeederDirection.STOPPED),
                                                Feeder.getInstance())));

                NamedCommands.registerCommand("VisionIntake", vision.PIDtoNoteRobotRelativeCommand(drivetrain::isPastCenterline).raceWith(new IntakeUntilLoadedCommand()));

                // NamedCommands.registerCommand("VisionIntake XY", vision.PIDtoNoteRobotRelativeCommand_XandYOnly(drivetrain::isPastCenterline).raceWith(new IntakeUntilLoadedCommand()));

                // NamedCommands.registerCommand("VisionIntake XVel", drivetrain.chasePoseRobotRelativeCommand_Y_WithXSupplier(vision::getRobotRelativeNotePose2d, 
                //                                                                 () -> drivetrain.getChassisSpeeds().vxMetersPerSecond, drivetrain::isPastCenterline).raceWith(new IntakeUntilLoadedCommand()));

                // extract gui composition to use just OptionalVisionIntakeCommand
                NamedCommands.registerCommand("SkipNoteLogic", new ShouldSkipNoteLogicCommand());
                NamedCommands.registerCommand("Backtrack", drivetrain.navigate(() -> poseEstimator.getStoredPose().get()));

                NamedCommands.registerCommand("AlignToSpeaker", new SequentialCommandGroup(
                        new RepeatCommand(new InstantCommand(() -> drivetrain.setSpeakerAlignAngle(() -> getTarget.get())).
                        until(() -> drivetrain.atSpeakerAlignAngle())),
                        new InstantCommand(() -> drivetrain.setSpeakerAlignAngle(null))));


                // NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
                // NamedCommands.registerCommand("SpoolShooter", new PrintCommand("Spooling"));
                // NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot"));
                // NamedCommands.registerCommand("StopShooter", new PrintCommand("Stop
                // Spooling"));
        }
}
