package frc.robot.subsystems.turret;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.constants.TurretConstants;

public class TurretIOSparkMax implements TurretIO {

    private final SparkMax turretMotor;
    private final RelativeEncoder turretEncoder;
    private final SparkClosedLoopController turretPID;

    private Rotation2d currentTargetAngle = new Rotation2d();

    public TurretIOSparkMax() {
        turretMotor   = new SparkMax(TurretConstants.kMotorId, MotorType.kBrushless);
        turretEncoder = turretMotor.getEncoder();
        turretPID     = turretMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();

        // Factores de conversión (gear ratio 20:1
        config.encoder.positionConversionFactor(TurretConstants.kPositionFactor);
        config.encoder.velocityConversionFactor(TurretConstants.kVelocityFactor);

        // Configuración general del motor
        config
            .idleMode(IdleMode.kBrake)
            .inverted(TurretConstants.kMotorInverted)
            .smartCurrentLimit(TurretConstants.kCurrentLimit)
            .voltageCompensation(TurretConstants.kMaxVolts);

        // limits  (±85°) 
        config.softLimit
            .forwardSoftLimit(TurretConstants.kUpperLimit).forwardSoftLimitEnabled(true)
            .reverseSoftLimit(TurretConstants.kLowerLimit).reverseSoftLimitEnabled(true);

        // PID 
        config.closedLoop
            .pid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
            .outputRange(TurretConstants.kMinOutput, TurretConstants.kMaxOutput);

        // FeedForward (kS/kV/kA) 
        config.closedLoop.feedForward
            .kS(TurretConstants.kS)
            .kV(TurretConstants.kV)
            .kA(TurretConstants.kA);

        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(TurretInputs inputs) {
        // El '-' invierte el signo del encoder, esto es en turretEnconder
        inputs.angle        = Rotation2d.fromRotations(turretEncoder.getPosition());
        inputs.targetAngle  = currentTargetAngle;
        inputs.velocityRPS  = turretEncoder.getVelocity();
        inputs.appliedVolts = turretMotor.getAppliedOutput() * turretMotor.getBusVoltage();
        inputs.current      = turretMotor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    @Override
    public void setSpeed(double dutyCycle) {
        turretMotor.set(dutyCycle);   // -1 a 1
    }

    @Override
    public void setPosition(Rotation2d position) {
        currentTargetAngle = position;
        turretPID.setReference(position.getRotations(), ControlType.kPosition);
    }

    @Override
    public void setPositionWithFF(Rotation2d position, double arbFFVolts) {
        currentTargetAngle = position;
        turretPID.setReference(
            position.getRotations(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            arbFFVolts,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void stop() {
        turretMotor.stopMotor();
    }

    @Override
    public void resetEnc() {
        turretEncoder.setPosition(0);
    }
}