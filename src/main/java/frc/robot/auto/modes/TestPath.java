package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class TestPath extends SequentialCommandGroup {
  public TestPath() {
    String path1 = "Amp Start To Top Note";
    String path2 = "Top To Bottom Tripple";
    String path3 = "Source To Lower Midfield";

    var maxswerve = Drivetrain.getInstance();
    addCommands(
        // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.MID),
        // new RunCommand(() -> elevatorArm.moveElevator(0.7))
        //             .until(() -> elevatorArm.getTopSwitch()),
        // new RunCommand(() -> motorIntake.autoMoveIntake(false)).withTimeout(1.0),
        // new InstantCommand(() -> motorIntake.setSpeed(0)),
        // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.SUBSTATION)
        //             .alongWith(new RunCommand(() -> elevatorArm.moveElevator(-0.7))
        //                         .until(() -> elevatorArm.getBottomSwitch())),
        // elevatorArm.movePivotCommand(() -> ElevatorMap.PivotPosition.DEFAULT),
        maxswerve.followPathCommand(path1, true));
        maxswerve.followPathCommand(path2, false);
        maxswerve.followPathCommand(path3, false);
  }
}
