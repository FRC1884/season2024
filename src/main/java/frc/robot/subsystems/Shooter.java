package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShooterMap;
import frc.robot.subsystems.Intake.IntakeStatus;

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
    private CANSparkBase top, bot;
    private CANSparkBase feeder;
    private SparkPIDController topPID, botPID, feedPID;

    private double leadVel, followVel, feedVel;
    private DigitalInput beamBreak;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shamper");

    public static enum NoteStatus {
        EMPTY, LOADED
    }

    public static enum FeederDirection{
        FORWARD, REVERSE, STOPPED
    }

    private NoteStatus status = NoteStatus.EMPTY;

    private Shooter() {
        if (ShooterMap.TOP_SHOOTER != -1) {
            top = new CANSparkFlex(ShooterMap.TOP_SHOOTER, MotorType.kBrushless);
            topPID = top.getPIDController();
            topPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            topPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            topPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            topPID.setFF(ShooterMap.FLYWHEEL_FF);

            top.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);

        }
        if (ShooterMap.BOTTOM_SHOOTER != -1) {
            bot = new CANSparkFlex(ShooterMap.BOTTOM_SHOOTER, MotorType.kBrushless);
            botPID = bot.getPIDController();
            botPID.setP(ShooterMap.FLYWHEEL_PID.kP);
            botPID.setI(ShooterMap.FLYWHEEL_PID.kI);
            botPID.setD(ShooterMap.FLYWHEEL_PID.kD);
            botPID.setFF(ShooterMap.FLYWHEEL_FF);

            bot.setClosedLoopRampRate(ShooterMap.FLYWHEEL_RAMP_RATE);
        }
        if (ShooterMap.FEEDER != -1){
            feeder = new CANSparkFlex(ShooterMap.FEEDER, MotorType.kBrushless);
            feedPID = feeder.getPIDController();
            feedPID.setP(ShooterMap.FEEDER_PID.kP);
            feedPID.setI(ShooterMap.FEEDER_PID.kI);
            feedPID.setD(ShooterMap.FEEDER_PID.kD);
            feedPID.setFF(ShooterMap.FEEDER_FF);

            feeder.setClosedLoopRampRate(ShooterMap.FEEDER_RAMP_RATE);
            feeder.setInverted(false);
        }

        beamBreak = new DigitalInput(ShooterMap.BEAMBREAK);
    }

    public double getSetpoint() {
        return top.getEncoder().getPosition();
    }

    public boolean isNoteLoaded(){
        //Once tripped 
        return status == NoteStatus.LOADED;
    }

    public void runFeeder(){
        feedVel = ShooterMap.FEEDER_RPM;
    }

    public void setFeederState(FeederDirection direction){
        switch (direction){
            case FORWARD:
            feedVel = ShooterMap.FEEDER_RPM;
            case REVERSE:
            feedVel = -ShooterMap.FEEDER_RPM;
            case STOPPED:
            feedVel = 0;
        }
    }

    public void stopFeeder(){}


    public Command setFlywheelVelocityCommand(double v) {
        return new InstantCommand(
                () -> {
                    leadVel = v;
                    followVel = v;
                });
    }

    public Command stopFlywheelCommand() {
        return new InstantCommand(
                () -> runFeeder());
    }




    @Override
    public void periodic() {
        status = (!beamBreak.get()) ? NoteStatus.EMPTY : NoteStatus.LOADED;
        updateMotors();
    }

    private void updateMotors() {
        if (top != null) {
            if (leadVel != 0) {
                topPID.setReference(leadVel, ControlType.kVelocity);
            } else
                top.set(0);
        }
        if (bot != null) {
            if (followVel != 0) {
                botPID.setReference(followVel, ControlType.kVelocity);
            } else
                bot.set(0);
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
        builder.addDoubleProperty("real top velo", () -> top.getEncoder().getVelocity(), (d) -> {
        });
        builder.addDoubleProperty("real bottom velo", () -> bot.getEncoder().getVelocity(), (d) -> {
        });
        builder.addDoubleProperty("real feeder velo", () -> feeder.getEncoder().getVelocity(), (d) -> {
        });
    }

}
