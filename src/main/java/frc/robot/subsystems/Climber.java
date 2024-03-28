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

    private Servo servo1, servo2;

    private Climber() {
        setName("Climber");
        var tab = Shuffleboard.getTab("Climber");
        tab.add(this);

        servo1 = new Servo(ClimberMap.SERVO_ID_1);
        servo2 = new Servo(ClimberMap.SERVO_ID_2);
    }

    private void setPosition(double pos){
        servo1.set(pos);
        servo2.set(pos);
    }

    public Command positionUp() {

        return Commands.runOnce(
            () -> {
                servo1.set(ClimberMap.TOP_VALUE);
                servo2.set(ClimberMap.TOP_VALUE);
            }
        ); // TODO: Change value 0 to highest position the climber can reach
    }

    public Command positionDown() {
        return Commands.runOnce(
            () -> {
                servo1.set(ClimberMap.LOCKED_VALUE);
                servo2.set(ClimberMap.LOCKED_VALUE);
            }
        );
    }


    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Climber Position", () -> servo1.get(), (s) -> setPosition(s));
    }
}
