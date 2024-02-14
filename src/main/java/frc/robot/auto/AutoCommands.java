package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain;

public class AutoCommands {
    public static void registerAutoCommands()
    {
     NamedCommands.registerCommand("Intake", new PrintCommand("Intaking now"));
     NamedCommands.registerCommand("Shoot", new PrintCommand("Shooting now"));
    }
}
