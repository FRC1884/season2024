package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShooterMap;

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
     * leaderPIVOT: LEFT PIVOT
     * followerPivot: RIGHT PIVOT
     */
    private CANSparkBase leaderFlywheel, followerFlywheel;
    private CANSparkBase feeder;
    private SparkPIDController leadPID, followPID, feedPID;

    private double leadVel, followVel, feedVel;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shamper");

    private Shooter() {
        if (ShooterMap.TOP_SHOOTER != -1) {
            leaderFlywheel = new CANSparkFlex(ShooterMap.TOP_SHOOTER, MotorType.kBrushless);
            leadPID = leaderFlywheel.getPIDController();
            leadPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            leadPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            leadPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            leadPID.setFF(ShooterMap.FLYWHEEL_FF);

            leaderFlywheel.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);

        }
        if (ShooterMap.BOTTOM_SHOOTER != -1) {
            followerFlywheel = new CANSparkFlex(ShooterMap.BOTTOM_SHOOTER, MotorType.kBrushless);
            followPID = followerFlywheel.getPIDController();
            followPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            followPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            followPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            followPID.setFF(ShooterMap.FLYWHEEL_FF);

            followerFlywheel.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
        }
        if (ShooterMap.FEEDER != -1)
            feeder = new CANSparkFlex(ShooterMap.FEEDER, MotorType.kBrushless);
        feedPID = feeder.getPIDController();
        feedPID.setP(ShooterMap.FEEDER_PID.kP);
        feedPID.setI(ShooterMap.FEEDER_PID.kI);
        feedPID.setD(ShooterMap.FEEDER_PID.kD);
        feedPID.setFF(ShooterMap.FEEDER_FF);

        feeder.setClosedLoopRampRate(ShooterMap.FEEDER_RAMP_RATE);

        feeder.setInverted(false);
    }

    public double getSetpoint() {
        return leaderFlywheel.getEncoder().getPosition();
    }

    public Command setFlywheelVelocityCommand(double v) {
        return new InstantCommand(
                () -> {
                    leadVel = v;
                    followVel = v;
                });
    }

    public Command stopFlywheelCommand() {
        return new InstantCommand(
                () -> {
                    leadVel = 0;
                    followVel = 0;
                });
    }

    public Command setFeederVelocityCommand(double v) {
        return new InstantCommand(() -> {
            feedVel = v;
        });
    }

    public Command stopFeederCommand() {
        return new InstantCommand(
                () -> {
                    feedVel = 0;
                });
    }


    @Override
    public void periodic() {
        updateMotors();
    }

    private void updateMotors() {
        if (leaderFlywheel != null) {
            if (leadVel != 0) {
                leadPID.setReference(leadVel, ControlType.kVelocity);
            } else
                leaderFlywheel.set(0);
        }
        if (followerFlywheel != null) {
            if (followVel != 0) {
                followPID.setReference(followVel, ControlType.kVelocity);
            } else
                followerFlywheel.set(0);
        }
        if (feeder != null) {
            if (feedVel != 0) {
                feedPID.setReference(feedVel, ControlType.kVelocity);
            } else
                feeder.set(0);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("lead velocity", () -> leadVel, (v) -> leadVel = v);
        builder.addDoubleProperty("follow velocity", () -> followVel, (v) -> followVel = v);
        // builder.addDoubleProperty("feeder velocity", () -> feedVel, (v) -> feedVel = v);
        builder.addDoubleProperty("real top velo", () -> leaderFlywheel.getEncoder().getVelocity(), (d) -> {
        });
        builder.addDoubleProperty("real bottom velo", () -> followerFlywheel.getEncoder().getVelocity(), (d) -> {
        });
        builder.addDoubleProperty("real feeder velo", () -> feeder.getEncoder().getVelocity(), (d) -> {
        });
    }

}
