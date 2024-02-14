package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotMap.Coordinates;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Drivetrain;

public class AutoCommands {
    public static void registerAutoCommands()
    {
     NamedCommands.registerCommand("Intake", new PrintCommand("Intake"));
     NamedCommands.registerCommand("Shoot", Drivetrain.getInstance()
     .alignCommand(() -> Coordinates.RED_SPEAKER.getTranslation()));
    }
}
