package frc.robot.core.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.core.TalonSwerve.SwerveConstants;

public final class CTREConfigs {
  public TalonFXConfiguration swerveAngleFXConfig;
  public TalonFXConfiguration swerveDriveFXConfig;
  public CANcoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit =
            new SupplyCurrentLimitConfiguration(
                    SwerveConstants.LIMIT_ROTATOR_CURRENT,
                    SwerveConstants.ROTATOR_CONTINUOS_CURRENT_LIMIT,
                    SwerveConstants.ROTATOR_PEAK_CURRENT_LIMIT,
                    SwerveConstants.ROTATOR_PEAK_CURRENT_DURATION);

    swerveAngleFXConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(SwerveConstants.ROTATOR_KP)
                    .withKI(SwerveConstants.ROTATOR_KI)
                    .withKD(SwerveConstants.ROTATOR_KD)
            ).withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(
                    angleSupplyLimit.currentLimit
            ));

    /* Swerve Drive Motor Configuration */
    SupplyCurrentLimitConfiguration driveSupplyLimit =
            new SupplyCurrentLimitConfiguration(
                    SwerveConstants.LIMIT_DRIVE_CURRENT,
                    SwerveConstants.DRIVE_CONTINUOS_CURRENT_LIMIT,
                    SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT,
                    SwerveConstants.DRIVE_PEAK_CURRENT_DURATION);

    swerveDriveFXConfig = new TalonFXConfiguration()
            .withSlot0(new Slot0Configs()
                    .withKP(SwerveConstants.ROTATOR_KP)
                    .withKI(SwerveConstants.ROTATOR_KI)
                    .withKD(SwerveConstants.ROTATOR_KD)
            ).withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(
                    angleSupplyLimit.currentLimit
            )).withOpenLoopRamps(new OpenLoopRampsConfigs()
                    .withVoltageOpenLoopRampPeriod(
                            SwerveConstants.OPEN_LOOP_RAMP
                    )
            ).withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(
                            SwerveConstants.CLOSED_LOOP_RAMP
                    )
            );

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig = new CANcoderConfiguration().withMagnetSensor(
            new MagnetSensorConfigs()
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );
  }
}
