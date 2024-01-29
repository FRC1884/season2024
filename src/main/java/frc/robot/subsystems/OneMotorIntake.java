package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import java.util.function.BooleanSupplier;


public class OneMotorIntake extends SubsystemBase {

    private final double INTAKE_SPEED = 0.5;
    private static OneMotorIntake instance;

    public static OneMotorIntake getInstance(){
        if (instance == null) instance = new OneMotorIntake();
        return instance;
    }
    CANSparkMax oneMotor;

    private OneMotorIntake() {
         oneMotor = new CANSparkMax(RobotMap.IntakeMap.MOTOR_ID, MotorType.kBrushless);

        oneMotor.setInverted(false);
    }


    public void run() {
        oneMotor.set(INTAKE_SPEED);
    }

    public void stop() {
        oneMotor.set(0);
    }
    
    public Command runMotor() {
        return new InstantCommand(this::run);
    }
    
    public Command stopMotor() {
        return new InstantCommand(this::stop);
    }


}