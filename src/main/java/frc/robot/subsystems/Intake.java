package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeMap;

public class Intake extends SubsystemBase {
    private static Intake instance;

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private CANSparkMax motor1, motor2;
    

    private Intake() {
        motor1 = new CANSparkMax(IntakeMap.MOTOR_ID_1, MotorType.kBrushless);
        motor2 = new CANSparkMax(IntakeMap.MOTOR_ID_2, MotorType.kBrushless);
    }

    public Command run(DoubleSupplier speed1,DoubleSupplier speed2) {
        return new RunCommand(()->{
            motor1.set(speed1.getAsDouble());
            motor2.set(speed2.getAsDouble());
        }, this);
    }
}
