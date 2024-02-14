package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.IntakeMap;
import frc.robot.RobotMap.PrototypeMap;

public class Intake extends SubsystemBase {
    private static Intake instance;
    

    public static Intake getInstance() {
        if(instance == null) {
            instance = new Intake();
        }
        return instance;
        
    }

    private CANSparkMax motor1, motor2;
    private double MOTOR_SPEED_1 = -0.8; //TODO: fix value
    private double MOTOR_SPEED_2 = -0.6;// TODO: fix value
    

    private Intake() {
        motor1 = new CANSparkMax(IntakeMap.MOTOR_ID_1, MotorType.kBrushless);
        //motor2 = new CANSparkMax(IntakeMap.MOTOR_ID_2, MotorType.kBrushless);
        
    }

    private void run(double speed1, double speed2, boolean yn){
        if(yn){
        motor1.set(speed1);
        // motor2.set(speed2);
    }
        else{ motor1.set(0.4);
        // motor2.set(0.4);
        }
    }

    public Command runCommand(boolean yn) {
        return new InstantCommand(
        ()->{
            run(MOTOR_SPEED_1, MOTOR_SPEED_2,yn);
        });
    }

    public Command stopCommand(){
        return new InstantCommand(
            () -> run(0, 0, true)
        ); 
    }

    
}
