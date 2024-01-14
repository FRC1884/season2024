package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.PrototypeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PrototypeTest extends SubsystemBase {

    private PrototypeTest instance;

    public PrototypeTest getInstance() {
        if (instance == null)
            instance = new PrototypeTest();
        return instance;
    }

    // Select any combination of these to initialise, and use the corresponding ID
    // numbers in robotmap
    private CANSparkMax motor1;
    private CANSparkMax motor2;
    private CANSparkMax motor3;
    private ShuffleboardTab prototypeTab = Shuffleboard.getTab("Prototyping Tab");

    private ShuffleboardLayout motor1Layout = Shuffleboard.getTab("Prototyping Tab")
            .getLayout("Motor 1", BuiltInLayouts.kList).withSize(2, 2)
            .withProperties(Map.of("Label Position", "HIDDEN")).withPosition(1, 1);

    private ShuffleboardLayout motor2Layout = Shuffleboard.getTab("Prototyping Tab")
            .getLayout("Motor 2", BuiltInLayouts.kList).withSize(2, 2)
            .withProperties(Map.of("Label Position", "HIDDEN")).withPosition(3, 1);

    private ShuffleboardLayout motor3Layout = Shuffleboard.getTab("Prototyping Tab")
            .getLayout("Motor 3", BuiltInLayouts.kList).withSize(2, 2)
            .withProperties(Map.of("Label Position", "HIDDEN")).withPosition(5, 1);

    private HashMap<CANSparkMax, ShuffleboardLayout> motorToLayout = new HashMap<CANSparkMax, ShuffleboardLayout>();

    private HashMap<CANSparkMax, HashMap<String, GenericEntry>> motorToEntry = new HashMap<CANSparkMax, HashMap<String, GenericEntry>>();

    private HashMap<String, GenericEntry> findEntry1 = new HashMap<String, GenericEntry>();
    private HashMap<String, GenericEntry> findEntry2 = new HashMap<String, GenericEntry>();
    private HashMap<String, GenericEntry> findEntry3 = new HashMap<String, GenericEntry>();

    private String[] dataEntryKeys = {
            "RunSpeed", "P", "I", "D", "FF", "setIZone", "AllowErr", "maxVel", "MaxAcc", "setpoint", "PIDEnable"
    };

    private PrototypeTest() {
        motor1 = new CANSparkMax(PrototypeMap.MOTOR_ID_1, MotorType.kBrushless);
        motor2 = new CANSparkMax(PrototypeMap.MOTOR_ID_2, MotorType.kBrushless);
        motor3 = new CANSparkMax(PrototypeMap.MOTOR_ID_3, MotorType.kBrushless);

        motorToLayout.put(motor1, motor1Layout);
        motorToLayout.put(motor2, motor2Layout);
        motorToLayout.put(motor3, motor3Layout);

        motorToEntry.put(motor1, findEntry1);
        motorToEntry.put(motor2, findEntry2);
        motorToEntry.put(motor3, findEntry3);

        configureMotorOnShuffleBoard(motor1);
        configureMotorOnShuffleBoard(motor2);
        configureMotorOnShuffleBoard(motor3);
        configureMotor(motor1);
        configureMotor(motor2);
        configureMotor(motor3);
    }

    /**
     * Adds both speed and PID values to configure onto shuffleboard
     * @param motor the motor to configure
     */
    private void configureMotorOnShuffleBoard(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        ShuffleboardLayout motorLayout = motorToLayout.get(motor);
        var motorEntry = motorToEntry.get(motor);
        motorEntry.put("RunSpeed", motorLayout.add("RunSpeed", 1).withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 1))
                .getEntry());
        motorEntry.put("P", motorLayout.add("P", 1).getEntry());
        motorEntry.put("I", motorLayout.add("I", 0).getEntry());
        motorEntry.put("D", motorLayout.add("D", 1).getEntry());
        motorEntry.put("FF", motorLayout.add("FF", 1).getEntry());
        motorEntry.put("AllowErr", motorLayout.add("Allowed Err", 0).getEntry());
        motorEntry.put("setIZone", motorLayout.add("setIZone", 0).getEntry());
        motorEntry.put("maxVel", motorLayout.add("Max Velocity", 2000).getEntry());
        motorEntry.put("maxAcc", motorLayout.add("MAx Acceleration", 1500).getEntry());
        motorEntry.put("setpoint", motorLayout.add("Setpoint", 0).getEntry());

        motorEntry.put("PIDEnable", motorLayout.add("PIDEnable", false).getEntry());
    }

    /**
     * Configures the basics for the motor using data from shuffleboard
     * 
     * @param motor the motor
     */
    private void configureMotor(CANSparkMax motor) {
        SparkPIDController motorPIDController = motor.getPIDController();
        var entry = motorToEntry.get(motor);
        motorPIDController.setP(entry.get("P").getDouble(1));
        motorPIDController.setI(entry.get("I").getDouble(0));
        motorPIDController.setD(entry.get("D").getDouble(0));
        motorPIDController.setFF(entry.get("FF").getDouble(0));
        motorPIDController.setIZone(entry.get("setIZone").getDouble(0));
        motorPIDController.setSmartMotionAllowedClosedLoopError(entry.get("AllowErr").getDouble(0), 0);
        motorPIDController.setSmartMotionMaxVelocity(entry.get("maxVel").getDouble(2000), 0);
        motorPIDController.setSmartMotionMaxAccel(entry.get("maxAcc").getDouble(1500), 0);
    }

    /**
     * Runs the motor to a certain setpoint defined on shuffleboard
     * @param motor motor to run to setpoint
     */
    private void runMotorToSetpoint(CANSparkMax motor) {
        var entry = motorToEntry.get(motor);
        double setpoint = entry.get("setpoint").getDouble(0);
        motor.getPIDController().setReference(setpoint, ControlType.kSmartMotion);
    }

    /**
     * @param motor  The base motor
     * @param parent the motor that the base motor is a follower of
     */
    private void configureMotor(CANSparkMax motor, CANSparkMax parent) {
        configureMotor(motor);
        motor.follow(parent);
    }

    /**
     * Checks if there is a change made on Shuffleboard to any of the motor values and changes it if there is. 
     * @param motor the motor to update the values for
     * @return
     */
    private void updateMotor(CANSparkMax motor) {
        var entry = motorToEntry.get(motor);
        for (String key : dataEntryKeys) {
            switch (key) {
                case "RunSpeed":
                    if (motor.get() != entry.get(key).getDouble(0))
                        motor.set(entry.get(key).getDouble(0));
                case "P":
                    if (motor.getPIDController().getP() != entry.get(key).getDouble(1))
                        motor.getPIDController().setP(entry.get(key).getDouble(1));
                case "I":
                    if (motor.getPIDController().getI() != entry.get(key).getDouble(0))
                        motor.getPIDController().setI(entry.get(key).getDouble(0));
                case "D":
                    if (motor.getPIDController().getD() != entry.get(key).getDouble(0))
                        motor.getPIDController().setD(entry.get(key).getDouble(0));
                case "FF":
                    if (motor.getPIDController().getFF() != entry.get(key).getDouble(0))
                        motor.getPIDController().setFF(entry.get(key).getDouble(0));
                case "setIZone":
                    if (motor.getPIDController().getIZone() != entry.get(key).getDouble(0))
                        motor.getPIDController().setIZone(entry.get(key).getDouble(0));
                case "maxVel":
                    if (motor.getPIDController().getSmartMotionMaxVelocity(0) != entry.get(key).getDouble(2000))
                        motor.getPIDController().setSmartMotionMaxVelocity(entry.get(key).getDouble(2000), 0);
                case "maxAcc":
                    if (motor.getPIDController().getSmartMotionMaxAccel(0) != entry.get(key).getDouble(1))
                        motor.getPIDController().setSmartMotionMaxAccel(entry.get(key).getDouble(1500), 0);
                case "AllowErr":
                    if (motor.getPIDController().getSmartMotionAllowedClosedLoopError(0) != entry.get(key).getDouble(0))
                        motor.getPIDController().setSmartMotionAllowedClosedLoopError(entry.get(key).getDouble(0), 0);
                case "PIDEnable":
                    if (entry.get(key).getBoolean(false))
                        runMotorToSetpoint(motor);

            }

        }
    }

    @Override
    public void periodic() {
        updateMotor(motor1);
        updateMotor(motor2);
        updateMotor(motor3);
    }
}
