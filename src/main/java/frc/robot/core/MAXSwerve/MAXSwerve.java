package frc.robot.core.MAXSwerve;

import static frc.robot.core.TalonSwerve.SwerveConstants.KINEMATICS;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathfindThenFollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Config;
import frc.robot.RobotMap;
import frc.robot.RobotMap.DriveMap;
import frc.robot.subsystems.PoseEstimator;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public abstract class MAXSwerve extends SubsystemBase {

    private double currentRotation = 0.0;
    private double currentTranslationDir = 0.0;
    private double currentTranslationMag = 0.0;

    private SlewRateLimiter magLimiter = new SlewRateLimiter(MaxSwerveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(MaxSwerveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private MAXSwerveModule fl, fr, bl, br;
    private Object gyro;
    private ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain Debugging");
    GenericEntry targetAngleEntry = tab.add("Target Angle", 0).getEntry();
    GenericEntry currentAngleEntry = tab.add("Current Angle", 0).getEntry();
    private double targetAngleTelemetry = 0;

    SwerveDriveOdometry odometry;

    //For Testing All functions
    private double startTime, currentTime;

    public MAXSwerve(
            MAXSwerveModule fl,
            MAXSwerveModule fr,
            MAXSwerveModule bl,
            MAXSwerveModule br) {

        switch (RobotMap.DriveMap.GYRO_TYPE) {
            case NAVX:
                gyro = new AHRS(SPI.Port.kMXP);
                break;
            case PIGEON:
                gyro = new Pigeon2(DriveMap.PIGEON_ID);
                break;
            default:
                break;
        }
        // gyro.getConfigurator().DefaultTimeoutSeconds = 50;
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        odometry = new SwerveDriveOdometry(
                MaxSwerveConstants.kDriveKinematics,
                getYawRot2d(),
                new SwerveModulePosition[]{
                        fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
                });
        AutoBuilder.configureHolonomic(this::getPose, this::startingOdometry, this::getChassisSpeeds, this::driveWithChassisSpeeds,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                        // in
                        // your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        MaxSwerveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the
                        // options
                        // here
                ), getShouldFlip(), this);

        setName("Swerve");
        var tab = Shuffleboard.getTab("Swerve");
        tab.add(this);
    }

    public Rotation2d getYawRot2d() {
        // TODO Auto-generated method stub
        return (MaxSwerveConstants.INVERT_GYRO)
                ? Rotation2d.fromDegrees(360 - getGyroYawDegrees())
                : Rotation2d.fromDegrees(getGyroYawDegrees());
    }

    public double getGyroYawDegrees() {
        // TODO Auto-generated method stub
        switch (RobotMap.DriveMap.GYRO_TYPE) {
            case NAVX:
                return -Double.valueOf(((AHRS) gyro).getYaw()); //Must be negative to CCW is positive
            case PIGEON:
                return ((Pigeon2) gyro).getYaw();
            default:
                return 0.0;
        }
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        //System.out.println(odometry.getPoseMeters());
        //System.out.println(MathUtil.inputModulus(this.getYaw().getDegrees(), -180, 180));
        odometry.update(
                getYawRot2d(),
                new SwerveModulePosition[]{
                        fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
                });
        //resetOdometry(PoseEstimator.getInstance().getPosition()); //NEEDS MORE TESTING
        targetAngleEntry.setDouble(targetAngleTelemetry);
        currentAngleEntry.setDouble(getHeading() % 360);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = fl.getPosition();
        positions[1] = fr.getPosition();
        positions[2] = bl.getPosition();
        positions[3] = br.getPosition();
        return positions;
    }

    public Pose2d getPose() {

        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                getYawRot2d(),
                new SwerveModulePosition[]{
                        fl.getPosition(), fr.getPosition(), bl.getPosition(), br.getPosition()
                },
                pose);
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(
            double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(MaxSwerveConstants.kDirectionSlewRate / currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = MAXSwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                currentTranslationDir = MAXSwerveUtils.StepTowardsCircular(
                        currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
                    // keep currentTranslationDir unchanged
                    currentTranslationMag = magLimiter.calculate(0.0);
                } else {
                    currentTranslationDir = MAXSwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
                    currentTranslationMag = magLimiter.calculate(inputTranslationMag);
                }
            } else {
                currentTranslationDir = MAXSwerveUtils.StepTowardsCircular(
                        currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                currentTranslationMag = magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
            ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
            currentRotation = rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            currentRotation = rot;
        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;

        double ySpeedDelivered = ySpeedCommanded * MaxSwerveConstants.kMaxSpeedMetersPerSecond;

        double rotDelivered = currentRotation * MaxSwerveConstants.kMaxAngularSpeed;
        var swerveModuleStates = MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered,
                        Rotation2d.fromDegrees(getHeading()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);

        fl.setDesiredState(swerveModuleStates[0]);
        fr.setDesiredState(swerveModuleStates[1]);
        bl.setDesiredState(swerveModuleStates[2]);
        br.setDesiredState(swerveModuleStates[3]);
    }

    public Command alignWhileDrivingCommand(Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Translation2d> target) {
        PIDController pid = new PIDController(0.01, 0, 0);
        pid.setTolerance(1.5);
        pid.enableContinuousInput(-180, 180);
        return new DeferredCommand(() ->
                new RepeatCommand(
                        new FunctionalCommand(
                                () -> {
                                    // Init
                                },
                                () -> {
                                    Translation2d currentTranslation = this.getPose().getTranslation();
                                    Translation2d targetVector = currentTranslation.minus(target.get());
                                    Rotation2d targetAngle = targetVector.getAngle();
                                    double newSpeed = pid.calculate(this.getGyroYawDegrees() + 180, targetAngle.getDegrees() + DriveMap.SPEAKER_ALIGN_OFFSET);
                                    this.drive(xSpeed.get(), ySpeed.get(),
                                            newSpeed, true, true);

                                },
                                interrupted -> {
                                    pid.close();
                                    this.drive(0.0, 0.0, 0.0, true, true);
                                },
                                pid::atSetpoint,
                                this)), Set.of(this)
        );
    }

    public void driveWithChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        var swerveModuleStates = MaxSwerveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
        fl.setDesiredState(swerveModuleStates[0]);
        fr.setDesiredState(swerveModuleStates[1]);
        bl.setDesiredState(swerveModuleStates[2]);
        br.setDesiredState(swerveModuleStates[3]);
    }

    public ChassisSpeeds getChassisSpeeds() {

        ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(fl.getState(), fr.getState(), bl.getState(), br.getState());

        return speeds;
    }

    public Command driveCommand(
            Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed) {
        return new RepeatCommand(
                new RunCommand(
                        () -> this.drive(xSpeed.get(), ySpeed.get(), rotSpeed.get(), true, true), this));
    }

    public BooleanSupplier getShouldFlip() {
        return () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }

            return false;
        };
    }

    public void startingOdometry(Pose2d startingPose) {
        //PoseEstimator.getInstance().resetPoseEstimate(startingPose);
        PoseEstimator.getInstance().resetPoseEstimate(new Pose2d(startingPose.getX(), startingPose.getY(), this.getYawRot2d()));
    }

    public Command followPathCommand(String pathName, boolean isFirstPath) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return followPathCommand(path, isFirstPath);
    }

    public Command followPathCommand(PathPlannerPath pathName, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> {
                            // Reset odometry for the first path you run during auto
                            if (isFirstPath && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                                PathPoint startingPoint = pathName.getPoint(0);
                                Pose2d startingPose = new Pose2d(
                                        startingPoint.position, this.getYawRot2d());
                                //VERY IMPORTANT SO THAT ODOMETRY IS NOT OVERRIDEN AS BEING THE ORIGIN
                                PoseEstimator.getInstance().resetPoseEstimate(startingPose);
                                this.resetOdometry(startingPose);
                            } else if (isFirstPath && DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                                PathPoint startingPoint = pathName.getPoint(0);
                                double startingX = RobotMap.VisionConfig.FIELD_LENGTH_METERS - startingPoint.position.getX();
                                Pose2d startingPose = new Pose2d(
                                        startingX, startingPoint.position.getY(), this.getYawRot2d()
                                );
                                PoseEstimator.getInstance().resetPoseEstimate(startingPose);
                            }
                        }),
                AutoBuilder.followPath(pathName).andThen(() -> System.out.println("Helo there")),
                new InstantCommand(() -> System.out.println(PoseEstimator.getInstance().getEstimatedPose())),
                new InstantCommand(() -> System.out.println(getPose()))
        );
    }

    public Command followUnflippedPathCommand(PathPlannerPath pathName) {
        return new FollowPathHolonomic(
                pathName,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveWithChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    /**
     * Command to drive to a note detected using Vision - NEEDS TO BE REWORKED TO USE NAVIGATE
     * Preivous ERROR: Robot travels to the robot relative note pose in field relative coordinates
     * FIX: PARAMETER MUST BE A SUPPLIER SO IT CONTINUOUSLY UPDATES
     *
     * @param targetPose the Supplier<Pose2d> that the robot should drive to
     * @return command to generate a path On-the-fly to a note
     */
    public Command onTheFlyPathCommand(Supplier<Pose2d> targetPose) {
        return new DeferredCommand(() -> followUnflippedPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(new Pose2d(this.getPose().getTranslation(),
                                        Rotation2d.fromDegrees(0)),
                                targetPose.get()),
                        new PathConstraints(
                                RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                                RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                                RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                                RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
                        new GoalEndState(0, targetPose.get().getRotation()),
                        false)),
                //.until(
                // () -> targetPose.get().getTranslation().getDistance(PoseEstimator.getInstance().getPosition().getTranslation())<1.5))
                //.finallyDo(() -> System.out.println("uwu owo"));
                Set.of(this));
    }

    /**
     * Command to pathfind to a path - NEEDS TO BE TESTED, also, does it need to be a proxy?
     *
     * @param pathName name of the path that the robot will pathfind to
     * @return command to generate a path to the starting pose of a premade path and then to follow that path
     */

    public Command pathFindThenFollowPathCommand(String pathName) {
        return new PathfindThenFollowPathHolonomic(
                PathPlannerPath.fromPathFile(pathName),
                new PathConstraints(
                        RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                        RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                        RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                        RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
                this::getPose,
                this::getChassisSpeeds,
                this::driveWithChassisSpeeds,
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
                        // in
                        // your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the
                        // options
                        // here
                ),
                1.0, // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate. Optional

                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                getShouldFlip(),

                this // Reference to drive subsystem to set requirements
        );
    }

    public Command navigateAndAlignCommand(String pathName, Supplier<Translation2d> target) {
        return new SequentialCommandGroup(
                pathFindThenFollowPathCommand(pathName),
                alignCommand(target)
        );
    }

    public Command alignCommand(Supplier<Translation2d> target) {
        PIDController pid = new PIDController(0.01, 0, 0);
        pid.setTolerance(1.5);
        pid.enableContinuousInput(-180, 180);
        return new DeferredCommand(() ->
                new FunctionalCommand(
                        () -> {
                            // Init
                        },
                        () -> {
                            Translation2d currentTranslation = this.getPose().getTranslation();
                            Translation2d targetVector = currentTranslation.minus(target.get());
                            Rotation2d targetAngle = targetVector.getAngle();
                            double newSpeed = pid.calculate(this.getGyroYawDegrees(), targetAngle.getDegrees());
                            this.drive(0, 0,
                                    newSpeed, true, true);

                            targetAngleEntry.setDouble(targetAngle.getDegrees());
                            currentAngleEntry.setDouble(this.getGyroYawDegrees());
                        },
                        interrupted -> {
                            pid.close();
                            //this.drive(0,0,0,true,true);
                            System.out.println("Allignment Over");
                        },
                        () -> {
                            return pid.atSetpoint();
                        },
                        this), Set.of(this)
        );
    }

    public Command navigate(Supplier<Pose2d> targetPose, Supplier<String> pathName) {
        return followPathCommand(
                new PathPlannerPath(
                        PathPlannerPath.bezierFromPoses(new Pose2d(this.getPose().getTranslation(),
                                        Rotation2d.fromDegrees(0)),
                                new Pose2d(targetPose.get().getTranslation(),
                                        Rotation2d.fromDegrees(0))),
                        new PathConstraints(
                                RobotMap.SwervePathFollowConstants.MAX_VELOCITY,
                                RobotMap.SwervePathFollowConstants.MAX_ACCELERATION,
                                RobotMap.SwervePathFollowConstants.MAX_ANG_VELOCITY,
                                RobotMap.SwervePathFollowConstants.MAX_ANG_ACCELERATION),
                        new GoalEndState(0, new Rotation2d()),
                        false),
                false);
    }

    /**
     * Command to go to a pose using a Trapezoidal PID profile for increased accuracy compared to a pure on the fly
     *
     * @param targetPose the Supplier<Pose2d> that the robot should drive to
     * @return command to PID align to a pose on the field
     */
    public Command chasePoseCommand(Supplier<Pose2d> target) {
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(1, 1.5);

        ProfiledPIDController xController = new ProfiledPIDController(0.1, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.1, 0, 0, Y_CONSTRAINTS);
        ProfiledPIDController omegaController = new ProfiledPIDController(0.01, 0, 0, OMEGA_CONSTRAINTS);

        xController.setTolerance(0.3);
        yController.setTolerance(0.3);
        omegaController.setTolerance(Units.degreesToRadians(3));
        omegaController.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
                new RepeatCommand(
                        new FunctionalCommand(
                                () -> {
                                    // Init
                                },
                                () -> {

                                    double xSpeed = xController.calculate(this.getPose().getX(), target.get().getX());
                                    double ySpeed = yController.calculate(this.getPose().getY(), target.get().getY());
                                    double omegaSpeed = omegaController.calculate(this.getGyroYawDegrees(), target.get().getRotation().getDegrees());

                                    this.drive(xSpeed, ySpeed, omegaSpeed, true, true);
                                },

                                interrupted -> {
                                    this.drive(0, 0, 0, true, true);
                                    System.out.println("30 cm away now");
                                },

                                () -> {
                                    return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
                                },
                                this)), Set.of(this)
        );
    }

    /**
     * Command to go to a pose using a Trapezoidal PID profile for increased accuracy compared to a pure on the fly
     *
     * @param targetPose the Supplier<Pose2d> that the robot should drive to ROBOT RELATIVE
     * @return command to PID align to a pose that is ROBOT RELATIVE
     */
    public Command chasePoseRobotRelativeCommand(Supplier<Pose2d> target) {
        TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
        //TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(1, 1.5);

        ProfiledPIDController xController = new ProfiledPIDController(0.5, 0, 0, X_CONSTRAINTS);
        ProfiledPIDController yController = new ProfiledPIDController(0.5, 0, 0, Y_CONSTRAINTS);
        PIDController omegaPID = new PIDController(0.01, 0, 0);

        xController.setTolerance(0.10);
        yController.setTolerance(0.03);
        omegaPID.setTolerance(1.5);
        omegaPID.enableContinuousInput(-180, 180);

        return new DeferredCommand(() ->
                new FunctionalCommand(
                        () -> {
                            // Init
                        },
                        () -> {

                            double xSpeed = xController.calculate(0, target.get().getX());
                            double ySpeed = yController.calculate(0, target.get().getY());
                            double omegaSpeed = omegaPID.calculate(0, target.get().getRotation().getDegrees());

                            this.drive(xSpeed, 0, omegaSpeed, false, true);
                        },

                        interrupted -> {
                            this.drive(0, 0, 0, false, true);
                            omegaPID.close();
                            System.out.println("Aligned now");
                        },

                        () -> {
                            return omegaPID.atSetpoint() && xController.atGoal() && yController.atGoal();
                        },
                        this), Set.of(this)
        );
    }

    /**
     * Command to drive using Trapezoidal PID to a set distance away from a pose, then on-the-fly path to the pose
     *
     * @param targetPose the Supplier<Pose2d> that the robot should drive to
     * @return command to PID align to and then on-the-fly path to a pose on the field
     */
    public Command chaseThenOnTheFlyCommand(Supplier<Pose2d> target) {
        return new SequentialCommandGroup(
                chasePoseCommand(target),
                onTheFlyPathCommand(target)
        );
    }

    // public Command goSpeakerOrSource(boolean hasNote) {
    //   if (hasNote) {
    //     return navigate(getPose(),"WithNote");
    //   } else {
    //     return navigate(getPose(), "NoNote");
    //   }
    // }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, MaxSwerveConstants.kMaxSpeedMetersPerSecond);
        fl.setDesiredState(desiredStates[0]);
        fr.setDesiredState(desiredStates[1]);
        bl.setDesiredState(desiredStates[2]);
        br.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        fl.resetEncoders();
        fr.resetEncoders();
        bl.resetEncoders();
        br.resetEncoders();
    }

    /**
     * Zeros the heading of the robot
     */
    public void zeroGyroYaw() {
        switch (RobotMap.DriveMap.GYRO_TYPE) {
            case NAVX:
                ((AHRS) gyro).reset();
                break;
            case PIGEON:
                ((Pigeon2) gyro).setYaw(0);
                break;
            default:
                break;
        }
    }

    /**
     * set the yaw of the gyro in degrees
     *
     * @param degrees
     */
    public void setGyroYaw(double degrees) {
        switch (RobotMap.DriveMap.GYRO_TYPE) {
            case NAVX:
                ((AHRS) gyro).setAngleAdjustment(degrees);
                break;
            case PIGEON:
                ((Pigeon2) gyro).setYaw(0);
                break;
            default:
                break;
        }
    }

    public Command zeroYawCommand() {
        return new InstantCommand(() -> zeroGyroYaw());
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return getGyroYawDegrees();
    }

    private void updateAlignAngle(double angle) {
        DriveMap.SPEAKER_ALIGN_OFFSET = angle;
    }

    private double getALignAngle() {
        return DriveMap.SPEAKER_ALIGN_OFFSET;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Manual Align offset", () -> getALignAngle(), (a) -> updateAlignAngle(a));
    }

    public SequentialCommandGroup TestAllCommand() {
        return new SequentialCommandGroup(
                new FunctionalCommand(
                        () -> {
                            startTime = Timer.getFPGATimestamp();
                        },
                        () -> {
                            drive(0.1, 0.1, 0, true, true);
                            currentTime = Timer.getFPGATimestamp();
                        },
                        interrupted -> {
                            drive(0, 0, 0, true, true);
                        },
                        () -> {
                            return (currentTime - startTime >= 0.5);
                        },
                        this
                ),
                new WaitCommand(1),
                new FunctionalCommand(
                        () -> {
                            startTime = Timer.getFPGATimestamp();
                        },
                        () -> {
                            drive(0, 0, 0.1, true, true);
                            currentTime = Timer.getFPGATimestamp();
                        },
                        interrupted -> {
                            drive(0, 0, 0, true, true);
                        },
                        () -> {
                            return (currentTime - startTime >= 0.5);
                        },
                        this
                ),
                new WaitCommand(1),

                navigate(() -> getPose().plus(new Transform2d(new Translation2d(1, 0), new Rotation2d())),
                        () -> "Testing Translation X"),
                new WaitCommand(1),
                navigate(() -> getPose().plus(new Transform2d(new Translation2d(0, 1), new Rotation2d())), () -> "Testing Translation Y"),
                new WaitCommand(1),
                navigate(() -> getPose().plus(new Transform2d(new Translation2d(0, 1), new Rotation2d(90))), () -> "Testing Rotation"),
                new WaitCommand(1),
                followPathCommand("ShortTestPath", true)
        );
    }
}
