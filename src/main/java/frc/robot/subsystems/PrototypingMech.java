package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PrototypeMap;

public class PrototypingMech extends SubsystemBase {

  private static PrototypingMech instance;

  public static PrototypingMech getInstance() {
    if (instance == null) instance = new PrototypingMech();
    return instance;
  }

  private CANSparkFlex flywheel1, flywheel2;
  private CANSparkMax max1, max2;

  private PrototypingMech() {
    flywheel1 = new CANSparkFlex(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
    flywheel2 = new CANSparkFlex(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
    max1 = new CANSparkMax(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);
    max2 = new CANSparkMax(PrototypeMap.MOTOR_ID_4, MotorType.kBrushless);
  }

  public Command run(double flywheelSpeed, double otherSpeed) {
    return new RunCommand(
        () -> {
          flywheel1.set(flywheelSpeed);
          flywheel2.set(-flywheelSpeed);
          max1.set(otherSpeed);
          max2.set(-otherSpeed);
        });
  }
}
