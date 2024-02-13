package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class SpeakerStartAllianceNotes extends SequentialCommandGroup {
  public SpeakerStartAllianceNotes() {
    String path1 = "Speaker To Bottom Note";
    String path2 = "Bottom To Top Tripple";
    String path3 = "Amp To Top Midfield";

    var maxswerve = Drivetrain.getInstance();
    addCommands(
        maxswerve.followPathCommand(path1, true),
        maxswerve.followPathCommand(path2, false),
        maxswerve.followPathCommand(path3, false));
  }
}
