// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.drivetrain;

// public class ChaseNoteCommand extends CommandBase {
  
//   private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//   private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
//   private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);
  
//   private static final Transform3d TAG_TO_GOAL = 
//       new Transform3d(
//           new Translation3d(1.5, 0.0, 0.0),
//           new Rotation3d(0.0, 0.0, Math.PI));

//   private final Drivetrain drivetrain;
//   private final Supplier<Pose2d> notePose;

//   private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
//   private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
//   private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);


//   public ChaseTagCommand( 
//         Drivetrain drivetrain,
//         Supplier<Pose2d> poseProvider) {
//     this.drivetrain = drivetrain.getInstance();
//     this.poseProvider = poseProvider;

//     xController.setTolerance(0.2);
//     yController.setTolerance(0.2);
//     omegaController.setTolerance(Units.degreesToRadians(3));
//     omegaController.enableContinuousInput(-Math.PI, Math.PI);

//     addRequirements(drivetrainSubsystem);
//   }

//   @Override
//   public void initialize() {
//     lastTarget = null;
//     var robotPose = drivetrain.getPose();
//     omegaController.reset(robotPose.getRotation().getRadians());
//     xController.reset(robotPose.getX());
//     yController.reset(robotPose.getY());
//   }

//   @Override
//   public void execute() {

//     if (photonRes.hasTargets()) {
//       // Find the tag we want to chase
//       var targetOpt = photonRes.getTargets().stream()
//           .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
//           .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
//           .findFirst();
//       if (targetOpt.isPresent()) {
//         var target = targetOpt.get();
//         // This is new target data, so recalculate the goal
//         lastTarget = target;
        
//         // Transform the robot's pose to find the camera's pose
//         var cameraPose = robotPose.transformBy(ROBOT_TO_CAMERA);

//         // Trasnform the camera's pose to the target's pose
//         var camToTarget = target.getBestCameraToTarget();
//         var targetPose = cameraPose.transformBy(camToTarget);
        
//         // Transform the tag's pose to set our goal
//         var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

//         // Drive
//         xController.setGoal(goalPose.getX());
//         yController.setGoal(goalPose.getY());
//         omegaController.setGoal(goalPose.getRotation().getRadians());
//       }
//     }
    
//     if (lastTarget == null) {
//       // No target has been visible
//       drivetrainSubsystem.stop();
//     } else {
//       // Drive to the target
//       var xSpeed = xController.calculate(robotPose.getX());
//       if (xController.atGoal()) {
//         xSpeed = 0;
//       }

//       var ySpeed = yController.calculate(robotPose.getY());
//       if (yController.atGoal()) {
//         ySpeed = 0;
//       }

//       var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//       if (omegaController.atGoal()) {
//         omegaSpeed = 0;
//       }

//       drivetrainSubsystem.drive(
//         ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation()));
//     }
//   }

//   @Override
//   public void end(boolean interrupted) {
//     drivetrainSubsystem.stop();
//   }

// }