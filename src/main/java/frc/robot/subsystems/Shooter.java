package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.RobotMap.ShooterMap;

public class Shooter {
    private CANSparkMax launchMotor; //declaration of launch motor
    private CANSparkMax feedMotor;// declaration of feed motor
    private static Shooter instance;

    private Shooter() {
        launchMotor = new CANSparkMax(ShooterMap.LAUNCH_MOTOR_ID, MotorType.kBrushless); // assigns launchmotor variable to CANSparkMax motor
        feedMotor = new CANSparkMax(ShooterMap.FEED_MOTOR_ID, MotorType.kBrushless);// assigns FeedMotor variable to anotherCANSparkMax motor
    }

    // making sure only one instance of shooter exists
    public static Shooter getInstance() {
        if (instance == null)
            instance = new Shooter();
        return instance;

    }
// setting launch motor speed
    public void setLaunchSpeed(double speed) {
        launchMotor.set(speed);

    }
// setting feed motor speed
    public void setFeedSpeed(double speed) {
        feedMotor.set(speed);
    }
// method for stopping both motors at the same time 
    public void stopMotors() {
        launchMotor.set(0);
        feedMotor.set(0);

    }
// sets motor speed to negative values so shooting mechanism can activate
    public Command runShooter() {
        return new StartEndCommand(
                () -> {
                    setLaunchSpeed(-ShooterMap.launchSpeed);
                    setFeedSpeed(-ShooterMap.feedSpeed);
                },
                () -> stopMotors());

    }
//preparing launch motor for shooting
    public Command prepareShootCommand() {
        return new InstantCommand(() -> setLaunchSpeed(ShooterMap.launchSpeed));

    }
// command for launching the note by activating the motors 
    public Command launchNoteCommand() {
        return new InstantCommand(
                () -> {
                    setLaunchSpeed(ShooterMap.launchSpeed);
                    setFeedSpeed(ShooterMap.feedSpeed);
                });
    }
}
