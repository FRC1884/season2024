package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.RobotMap.PrototypeMap;

/**
 * The {@link Sendable} implementation for motors that allows us to send an entire motor as a
 * package to Shuffleboard or Elastic. <br>
 * <br>
 * <b>For this to work, you MUST run the program in Test Mode on the Driver Station!</b>
 */
public class SendableMotor implements Sendable {
  private CANSparkBase motor;
  private SparkPIDController closedLoopController;

  public boolean openLoopEnabled = true, closedLoopEnabled = true;
  public double speed; 
  private double setpoint;
  private SlewRateLimiter sRL;
  private boolean isInverted = false;
  private double P = 0.0003;
  private double I = 0.0000008;
  private double D= 0.000006;


  public SendableMotor(CANSparkBase motor) {
    this.motor = motor;
    closedLoopController = motor.getPIDController();
    sRL = new SlewRateLimiter(3);
  }

  void enable(boolean enabled) {
    openLoopEnabled = enabled;
  }

  void enablePID(boolean enabled) {
    closedLoopEnabled = enabled;
  }

  void setSpeed(double speed) {
    this.speed = speed;
    // m_motorController.set(speed);
    if (openLoopEnabled) motor.set(speed);
  }

  void setP(double p) {
    P = p;
    if (openLoopEnabled && closedLoopEnabled) closedLoopController.setP(P);
  }

  void setI(double i) {
    I=i;
    if (openLoopEnabled && closedLoopEnabled) closedLoopController.setI(I);
  }

  void setD(double d) {
    D=d;
    if (openLoopEnabled && closedLoopEnabled) closedLoopController.setD(D);
  }

  void setFF(double ff) {
    if (openLoopEnabled && closedLoopEnabled) closedLoopController.setFF(ff);
  }

  void setIZone(double iZone) {
    if (openLoopEnabled && closedLoopEnabled) closedLoopController.setIZone(iZone);
  }

  void setMaxVel(double vel) {
    if (openLoopEnabled && closedLoopEnabled)
      closedLoopController.setSmartMotionMaxVelocity(vel, motor.getDeviceId());
  }

  void setMaxAccel(double accel) {
    if (openLoopEnabled && closedLoopEnabled)
      closedLoopController.setSmartMotionMaxAccel(accel, motor.getDeviceId());
  }

  void setTolerance(double tolerance) {
    if (openLoopEnabled && closedLoopEnabled)
      motor
          .getPIDController()
          .setSmartMotionAllowedClosedLoopError(tolerance, motor.getDeviceId());
  }

  void setSetpoint(double setpoint) {
    if (openLoopEnabled && closedLoopEnabled) {
      this.setpoint = setpoint;
    }
  }
  double getVelocity(){
    return (motor.getEncoder().getVelocity()*2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS)/60;
  }

  // TODO: fix isInverted so we don't do negative doubles
  void isInverted(boolean isInverted){ 
    isInverted= this.isInverted;
    if(isInverted)motor.setInverted(isInverted);
    else motor.setInverted(isInverted);
  }

  void setVelocity(double velocity) {}

  public void control() {
    if(openLoopEnabled) {
      if(closedLoopEnabled) {
        this.motor.getPIDController()
          .setReference((setpoint/(2 * (Math.PI)* PrototypeMap.WHEEL_RADIUS))*60,
            //trapezoidalSetpoint.velocity + closedLoopController.getFF() * setpoint, 
            ControlType.kVelocity);
      }
      
      else motor.set(speed);
    }

    else motor.set(0.0);
  }

  // Adds a bunch of control properties on shuffleboard
  // Do changes on the board affect behavior?
  // Do I need to goof around in periodic?
  // Only time will tell...
  @Override
  public void initSendable(SendableBuilder builder) {
    SparkPIDController closedLoopController = motor.getPIDController();
    builder.setActuator(false);

    builder.addBooleanProperty("Enabled", () -> openLoopEnabled, this::enable);
    builder.addBooleanProperty("isInverted", () -> isInverted, this::isInverted);
    builder.addBooleanProperty("PID Enabled", () -> closedLoopEnabled, this::enablePID);

    builder.addDoubleProperty("Speed", motor::get, this::setSpeed);

    builder.addDoubleProperty("Setpoint", ()->setpoint, this::setSetpoint);
    builder.addDoubleProperty("Velocity",  this::getVelocity, this::setVelocity);


    builder.addDoubleProperty("P", closedLoopController::getP, this::setP);
    builder.addDoubleProperty("I", closedLoopController::getI, this::setI);
    builder.addDoubleProperty("D", closedLoopController::getD, this::setD);
    builder.addDoubleProperty("FF", closedLoopController::getFF, this::setFF);

    
    builder.addDoubleProperty("MaxVel", closedLoopController::getD, this::setMaxVel);
    builder.addDoubleProperty("MaxAccel", closedLoopController::getFF, this::setMaxAccel);
  }
}
