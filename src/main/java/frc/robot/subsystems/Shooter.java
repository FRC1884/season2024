package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.Coordinates;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.util.ActionSetpoint;
import frc.robot.util.FlywheelLookupTable;

/**
 * This is the Catapul-- umm... Flywheel subsystem for the 2024 season.
 * Throughout the season, add
 * everything shooting here. <br>
 * <br>
 * Think:
 *
 * <ul>
 * <li>pitch control for adjusting the launch angle,
 * <li>LUT-based power setpoints,
 * <li>closed-loop control to regain rotational momentum quickly,
 * <li>whatever else you'd like!
 * </ul>
 */
public class Shooter extends SubsystemBase {
    private static Shooter instance;

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;
    }

    // Motor Controllers
    /*
     * leaderFlywheel: TOP FLYWHEEL
     * followerFlywheel: BOTTOM FLYWHEEL
     */
    private Optional<CANSparkBase> top, bot;

    private double leadVel, followVel;

    FlywheelLookupTable lookupTable = ShooterMap.SPEAKER_LOOKUP_TABLE;
    Supplier<Pose2d> target = () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == (DriverStation.Alliance.Blue) ? Coordinates.BLUE_SPEAKER : Coordinates.RED_SPEAKER;
    PoseEstimator poseEstimator = PoseEstimator.getInstance();

    private Shooter() {
        super();

        if (ShooterMap.TOP_SHOOTER != -1) {
            top = Optional.of(new CANSparkFlex(ShooterMap.TOP_SHOOTER, MotorType.kBrushless));

            var motor = top.get();

            motor.restoreFactoryDefaults();

            motor.setIdleMode(IdleMode.kCoast);
            var pid = motor.getPIDController();
            pid.setP(ShooterMap.FLYWHEEL_PID.kP);
            pid.setI(ShooterMap.FLYWHEEL_PID.kI);
            pid.setD(ShooterMap.FLYWHEEL_PID.kD);
            pid.setFF(ShooterMap.FLYWHEEL_FF);

            motor.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            motor.burnFlash();
        } else {
            top = Optional.empty();
        }

        if (ShooterMap.BOTTOM_SHOOTER != -1) {
            bot = Optional.of(new CANSparkFlex(ShooterMap.BOTTOM_SHOOTER, MotorType.kBrushless));

            var motor = bot.get();

            motor.restoreFactoryDefaults();

            motor.setIdleMode(IdleMode.kCoast);
            var pid = motor.getPIDController();
            pid.setP(ShooterMap.FLYWHEEL_PID.kP);
            pid.setI(ShooterMap.FLYWHEEL_PID.kI);
            pid.setD(ShooterMap.FLYWHEEL_PID.kD);
            pid.setFF(ShooterMap.FLYWHEEL_FF);

            motor.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
            motor.burnFlash();
        } else {
            bot = Optional.empty();
        }
    }

    public Command setFlywheelVelocityCommand(Supplier<Double> v) {
        return new RunCommand(
                () -> {
                    leadVel = v.get();
                    followVel = v.get();
                }, this);
    }

    public Command stopFlywheelCommand() {
        return setFlywheelVelocityCommand(() -> 0.0);
    }

    public boolean getFlywheelIsAtVelocity() {
        if (top.isEmpty() || bot.isEmpty())
            return false;

        return Math.abs(top.get().getEncoder().getVelocity() - leadVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE
                && Math.abs(bot.get().getEncoder().getVelocity() - followVel) < ShooterMap.FLYWHEEL_VELOCITY_TOLERANCE;
    }


    @Override
    public void periodic() {
        updateMotors();
    }

    private void updateMotors() {
        if (top.isPresent()) {
            if (leadVel != 0) {
                top.get().getPIDController().setReference(leadVel, ControlType.kVelocity);
            } else
                top.get().set(0);
        }
        if (bot.isPresent()) {
            if (followVel != 0) {
                bot.get().getPIDController().setReference(followVel, ControlType.kVelocity);
            } else
                bot.get().set(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("lead velocity", () -> leadVel, (v) -> leadVel = v);
        builder.addDoubleProperty("follow velocity", () -> followVel, (v) -> followVel = v);
        builder.addDoubleProperty("real top velo",
                () -> top.map(motor -> motor.getEncoder().getVelocity()).orElse(0.0),
                (d) -> {}
        );
        builder.addDoubleProperty("real bot velo",
                () -> bot.map(motor -> motor.getEncoder().getVelocity()).orElse(0.0),
                (d) -> {}
        );

        Supplier<ActionSetpoint> getSetpoint = () -> lookupTable.get(poseEstimator.getDistanceToPose(target.get().getTranslation()));

        builder.addDoubleProperty("lookup rpm", () -> getSetpoint.get().getRPM(), (d) -> {
        });
        builder.addDoubleProperty("lookup angle", () -> getSetpoint.get().getAngle(), (d) -> {
        });
    }

}
