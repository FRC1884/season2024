package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotMap;
import frc.robot.RobotMap.ClimberMap;
import frc.robot.RobotMap.ShooterMap;

public class Climber extends SubsystemBase {
    private static Climber instance;

    public static Climber getInstance() {
        if(instance == null) instance = new Climber();
        return instance;
    }

    private ProfiledPIDController leaderProfile, followerProfile;
    private CANSparkBase leaderMotor, followerMotor;
    private Servo servo1, servo2;
    private boolean lockState;

    private Climber() {
        setName("Climber");
        var tab = Shuffleboard.getTab("Climber");
        tab.add(this);

        leaderMotor = new CANSparkMax(RobotMap.ClimberMap.LEADER_ID, MotorType.kBrushless);
        followerMotor = new CANSparkMax(RobotMap.ClimberMap.FOLLOWER_ID, MotorType.kBrushless);

        // leaderMotor.restoreFactoryDefaults();
        // followerMotor.restoreFactoryDefaults();

        leaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        followerMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        leaderMotor.setSmartCurrentLimit(40);
        followerMotor.setSmartCurrentLimit(40);

        // leaderMotor.setInverted(false);

        followerMotor.follow(leaderMotor, true);
        leaderMotor.burnFlash();
        followerMotor.burnFlash();
        //followerMotor.setInverted(true);

        servo1 = new Servo(ClimberMap.SERVO_ID_1);
        servo2 = new Servo(ClimberMap.SERVO_ID_2);

        lockState = false;
    }


    private void setPositions(double pos){
        if(servo1.get() != pos){
            servo1.set(pos);
        }
        if(servo2.get() != pos){
            servo2.set(pos);
        }
    }

    public Command toggleClimberLockStatusCommand() {

        return Commands.runOnce(
            () -> {
                lockState = !lockState;
            }
        );
    }

    public Command run(DoubleSupplier power) {
        return new InstantCommand(()->leaderMotor.set(power.getAsDouble()), this);
    }

    public SequentialCommandGroup alignSequence(){
        return new SequentialCommandGroup(positionUp(), new WaitCommand(5), positionDown()); // TODO: Change for time it takes to bring climber all the way up
    }

    public Command positionUp() {
        return Commands.runOnce(
            () -> {
                leaderProfile.setGoal(0); // TODO: Change value 0 to highest position the climber can reach
                followerProfile.setGoal(0);
            }
        ); // TODO: Change value 0 to highest position the climber can reach
    }
    public Command positionDown() {
        return Commands.runOnce(
            () -> {
                leaderProfile.setGoal(0); // TODO: Change value 0 to lowest position the climber can reach
                followerProfile.setGoal(0); // TODO: Change value 0 to lowest position the climber can reach
            }
        );
    }

    @Override
    public void periodic(){
        if(lockState){
            setPositions(ClimberMap.TOP_VALUE);
        } else if (!lockState){
            setPositions(ClimberMap.LOCKED_VALUE);
        }
    }


    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Climber Position", () -> servo1.get(), (s) -> setPositions(s));
    }
}
