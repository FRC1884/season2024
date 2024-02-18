package frc.robot.auto;

import java.time.Instant;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotMap.Coordinates;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.util.FlywheelLookupTable;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Shamper;

public class AutoCommands {
    public static void registerAutoCommands() {
        FlywheelLookupTable lookupTable = FlywheelLookupTable.getInstance();
        Pose2d target = DriverStation.getAlliance().get() == (DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER
                : Coordinates.RED_SPEAKER;
        PoseEstimator poseEstimator = PoseEstimator.getInstance();
        Pivot pivot = Pivot.getInstance();
        Shamper shooter = Shamper.getInstance();
        Intake intake = Intake.getInstance();
        NamedCommands.registerCommand("SpoolShooter", shooter.setShootVelocityCommand(() -> lookupTable.get(
                poseEstimator.getDistanceToPose(target.getTranslation())).getFlywheelV(),
                () -> lookupTable.get(poseEstimator.getDistanceToPose(target.getTranslation())).getFeederV()));
                
        NamedCommands.registerCommand("Pivot", pivot.updatePosition(() -> lookupTable
                .get(poseEstimator.getDistanceToPose(target.getTranslation())).getAngleSetpoint()));
        NamedCommands.registerCommand("Intake", intake.intakeUntilLoadedCommand());
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
                new InstantCommand(() -> intake.setIntakeState(IntakeDirection.FORWARD), intake),
                new WaitCommand(0.5),
                new InstantCommand(() -> intake.setIntakeState(IntakeDirection.STOPPED), intake)));
        NamedCommands.registerCommand("VisionIntake",
                intake.intakeUntilLoadedCommand().alongWith(Vision.getInstance().followNoteCommand().onlyIf(
                        () -> !Vision.getInstance().getNotePose2d().getTranslation().equals(new Translation2d(0, 0)))));
        // NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
        // NamedCommands.registerCommand("SpoolShooter", new PrintCommand("Spooling"));
        // NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot"));
        // NamedCommands.registerCommand("StopShooter", new PrintCommand("Stop
        // Spooling"));
    }
}
