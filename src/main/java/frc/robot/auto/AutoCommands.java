package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotMap.Coordinates;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Drivetrain;

public class AutoCommands {
    public static void registerAutoCommands()
    {
        //NamedCommands.registerCommand("Intake", Intake.getInstance().intakeUntilLoadedCommand());
    //     NamedCommands.registerCommand("SpoolShooter", Shooter.getInstance().setFlywheelVelocityCommand(6000));
    //     NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
    //         Shooter.getInstance().setFeederVelocityCommand(6000),
    //         new WaitCommand(1),
    //         Shooter.getInstance().setFeederVelocityCommand(0)));
    //     NamedCommands.registerCommand("StopShooter", Shooter.getInstance().setFlywheelVelocityCommand(0));
    //  NamedCommands.registerCommand("IntakeStart", Intake.getInstance().setIntakeState(IntakeDirection.FORWARD));
    //  NamedCommands.registerCommand("IntakeStop", Intake.getInstance().setIntakeState(IntakeDirection.STOPPED));
    //  NamedCommands.registerCommand("ShootStart", Shooter.getInstance().setFlywheelVelocityCommand(4000));
    NamedCommands.registerCommand("VisionIntake", Vision.getInstance().followNoteCommand().onlyIf(
        () -> !Vision.getInstance().getNotePose2d().getTranslation().equals(new Translation2d(0,0))));
    NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
    NamedCommands.registerCommand("SpoolShooter", new PrintCommand("Spooling"));
    NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot"));
    NamedCommands.registerCommand("StopShooter", new PrintCommand("Stop Spooling"));
    }
}
