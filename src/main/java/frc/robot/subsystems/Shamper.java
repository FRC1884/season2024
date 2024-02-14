package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.ShamperMap;
import frc.robot.util.SendableMotor;
import frc.robot.RobotMap.PivotMap;

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
public class Shamper extends SubsystemBase {
    private static Shamper instance;

    public static Shamper getInstance() {
        if (instance == null)
            instance = new Shamper();
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
    private SparkPIDController leaderFlywheel_PIDController, followerFlywheel_PIDController, pivot_PIDController;
    private TrapezoidProfile profile1;

    private double goalPos, goalVel;

    private SparkLimitSwitch pivotReverseLimitSwitch, pivotForwardLimitSwitch;

    // private RelativeEncoder pivotEncoder;

    private SendableMotor pivotSendable;
    private ShuffleboardTab tab = Shuffleboard.getTab("Shamper");
    private GenericEntry pivEntry = tab.add("enc", 0.0).getEntry();
    private GenericEntry goalVelocityEntry = tab.add("Goal Velocity", 0.0).getEntry();
    private GenericEntry curVelocityEntry = tab.add("Current Velocity", 0.0).getEntry();
    
    private final DecimalFormat df = new DecimalFormat();


    private Shamper() {
        if (ShamperMap.TOP_SHOOTER != -1) {
            leaderFlywheel = new CANSparkFlex(ShamperMap.TOP_SHOOTER, MotorType.kBrushless);
            leaderFlywheel_PIDController = leaderFlywheel.getPIDController();
            leaderFlywheel_PIDController.setP(ShamperMap.FLYWHEEL_PID.kP);
            leaderFlywheel_PIDController.setI(ShamperMap.FLYWHEEL_PID.kI);
            leaderFlywheel_PIDController.setD(ShamperMap.FLYWHEEL_PID.kD);
            leaderFlywheel_PIDController.setFF(ShamperMap.FLYWHEEL_FF);
        }
        if (ShamperMap.BOTTOM_SHOOTER != -1) {
            followerFlywheel = new CANSparkFlex(ShamperMap.BOTTOM_SHOOTER, MotorType.kBrushless);
            followerFlywheel_PIDController = followerFlywheel.getPIDController();
            followerFlywheel_PIDController.setP(ShamperMap.FLYWHEEL_PID.kP);
            followerFlywheel_PIDController.setI(ShamperMap.FLYWHEEL_PID.kI);
            followerFlywheel_PIDController.setD(ShamperMap.FLYWHEEL_PID.kI);
            followerFlywheel_PIDController.setFF(ShamperMap.FLYWHEEL_FF);
        }
        if (ShamperMap.FEEDER != -1)
            feeder = new CANSparkFlex(ShamperMap.FEEDER, MotorType.kBrushless);

        feeder.setInverted(false);
    }

    public double getSetpoint() {
        return leaderFlywheel.getEncoder().getPosition();
    }

    public Command runFlywheel(double setpoint) {
        SlewRateLimiter sRL = new SlewRateLimiter(0.3, -0.3, leaderFlywheel.getEncoder().getVelocity());
        return new InstantCommand(
                () -> {
                  goalVelocityEntry.setDouble(rpmToMetersPS(setpoint));
                  //goalVel = rpmToMetersPS(setpoint);
                    leaderFlywheel.getPIDController().setReference(rpmToMetersPS(setpoint),
                            CANSparkBase.ControlType.kVelocity);
                    followerFlywheel.getPIDController().setReference(rpmToMetersPS(setpoint),
                            CANSparkBase.ControlType.kVelocity);
                });
    }

    public Command runFlywheelPower(double power){
        return new InstantCommand(() -> leaderFlywheel.set(power));
    }

    public Command stopFlywheel(){
        return new InstantCommand(() -> leaderFlywheel.set(0));
    }

    public Command runFeeder(double power) {
        SlewRateLimiter sRL = new SlewRateLimiter(0.3);
        return new InstantCommand(
                () -> {
                    feeder.set(sRL.calculate(power));
                });
    }

    public Command runFeederPower(double power, boolean placeHolder){
        return new InstantCommand(() -> feeder.set(power));
    }

    private static double rpmToMetersPS(double value) {
        return (value * 60) / (2 * (Math.PI) * ShamperMap.FLYWHEEL_RADIUS);
    }

    @Override
    public void periodic() {
      curVelocityEntry.setDouble(leaderFlywheel.getEncoder().getVelocity());
                
            // SmartDashboard.putNumber("pivot enc", pivotEncoder.getPosition());
    }
}