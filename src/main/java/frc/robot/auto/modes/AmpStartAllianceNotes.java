package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AmpStartAllianceNotes extends SequentialCommandGroup {
  public AmpStartAllianceNotes() {
    String path1 = "Amp Start To Top Note";
    String path2 = "Top To Bottom Tripple Less Rotating";
    String path3 = "Source To Lower Midfield";

    var maxswerve = Drivetrain.getInstance();
    addCommands(
        maxswerve.followPathCommand(path1, true),
        maxswerve.followPathCommand(path2, false),
        maxswerve.followPathCommand(path3, false));
  }
}
