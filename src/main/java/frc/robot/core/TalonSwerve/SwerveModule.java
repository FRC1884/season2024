package frc.robot.core.TalonSwerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.core.util.Conversions;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;

  private TalonFX rotator;
  private TalonFX drive;
  private CANcoder angleEncoder;

  SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          SwerveConstants.DRIVE_KS, SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA);

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.encoderId);
    configAngleEncoder();

    /* Angle Motor Config */
    rotator = new TalonFX(moduleConstants.rotatorId);
    configAngleMotor();

    /* Drive Motor Config */
    drive = new TalonFX(moduleConstants.driveId);
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / SwerveConstants.MAX_VELOCITY;
      drive.set(percentOutput);
    } else {
      double velocity =
          Conversions.MPSToFalcon(
              desiredState.speedMetersPerSecond,
              SwerveConstants.WHEEL_CIRCUMFERENCE,
              SwerveConstants.DRIVE_GEAR_RATIO);
      drive.setControl(new VelocityVoltage(velocity));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.MAX_VELOCITY * 0.01))
            ? lastAngle
            : desiredState
                .angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    rotator.setControl(
            new PositionVoltage(
                    Conversions.degreesToFalcon(
                            angle.getDegrees(), SwerveConstants.ANGLE_GEAR_RATIO
                    )
            )
    );
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.falconToDegrees(
            rotator.getPosition().getValue(), SwerveConstants.ANGLE_GEAR_RATIO));
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValue());
  }

  public void resetToAbsolute() {
    double absolutePosition =
        Conversions.degreesToFalcon(
            getCanCoder().getDegrees() - angleOffset.getDegrees(),
            SwerveConstants.ANGLE_GEAR_RATIO);
    rotator.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.getConfigurator().DefaultTimeoutSeconds = 50;
    angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    rotator.getConfigurator().DefaultTimeoutSeconds = 50;
    rotator.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
    rotator.setInverted(SwerveConstants.ANGLE_MOTOR_INVERT);
    rotator.setNeutralMode(NeutralModeValue.Coast);
    resetToAbsolute();
  }

  private void configDriveMotor() {
    drive.getConfigurator().DefaultTimeoutSeconds = 50;
    drive.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
    drive.setInverted(SwerveConstants.DRIVE_MOTOR_INVERT);
    drive.setNeutralMode(NeutralModeValue.Brake);
    drive.setPosition(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.falconToMPS(
            drive.getVelocity().getValue(),
            SwerveConstants.WHEEL_CIRCUMFERENCE,
            SwerveConstants.DRIVE_GEAR_RATIO),
        getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        Conversions.falconToMeters(
            drive.getPosition().getValue(),
            SwerveConstants.WHEEL_CIRCUMFERENCE,
            SwerveConstants.DRIVE_GEAR_RATIO),
        getAngle());
  }
}
