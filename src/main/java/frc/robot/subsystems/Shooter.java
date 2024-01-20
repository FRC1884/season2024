package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RobotMap.ShooterMap;

public class Shooter {
    private CANSparkMax launchMotor;
    private CANSparkMax feedMotor;
    private static Shooter instance;

    private Shooter() {
        launchMotor = new CANSparkMax(ShooterMap.LAUNCH_MOTOR_ID, MotorType.kBrushless);
        feedMotor = new CANSparkMax(ShooterMap.FEED_MOTOR_ID, MotorType.kBrushless);
    }

    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;

    }

    public void setLaunchSpeed(double speed) {
        launchMotor.set(speed);

    }

    public void setFeedSpeed(double speed) {
        feedMotor.set(speed);
    }

    public void stopMotors() {
        launchMotor.set(0);
        feedMotor.set(0);

    }

    public Command runShooter() {
        return new StartEndCommand(
                () -> {
                    setLaunchSpeed(-ShooterMap.launchSpeed);
                    setFeedSpeed(-ShooterMap.feedSpeed);
                },
                () -> stopMotors());

    }

    public Command prepareShootCommand() {
        return new InstantCommand(() -> setLaunchSpeed(ShooterMap.launchSpeed));

    }

    public Command launchNoteCommand() {
        return new InstantCommand(
                () -> {
                    setLaunchSpeed(ShooterMap.launchSpeed);
                    setFeedSpeed(ShooterMap.feedSpeed);
                });
    }
}
