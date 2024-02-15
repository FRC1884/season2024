package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotMap.Coordinates;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shamper;
import frc.robot.subsystems.Intake.IntakeDirection;
import frc.robot.subsystems.Drivetrain;

public class AutoCommands {
    public static void registerAutoCommands()
    {
        //NamedCommands.registerCommand("Intake", Intake.getInstance().intakeUntilLoadedCommand());
        NamedCommands.registerCommand("SpoolShooter", Shamper.getInstance().setFlywheelVelocityCommand(6000));
        NamedCommands.registerCommand("Shoot", new SequentialCommandGroup(
            Shamper.getInstance().setFeederVelocityCommand(6000),
            new WaitCommand(1),
            Shamper.getInstance().setFeederVelocityCommand(0)));
        NamedCommands.registerCommand("StopShooter", Shamper.getInstance().setFlywheelVelocityCommand(0));
     NamedCommands.registerCommand("IntakeStart", Intake.getInstance().setIntakeState(IntakeDirection.FORWARD));
     NamedCommands.registerCommand("IntakeStop", Intake.getInstance().setIntakeState(IntakeDirection.STOPPED));
     NamedCommands.registerCommand("ShootStart", Shamper.getInstance().setFlywheelVelocityCommand(4000));
    }
}
