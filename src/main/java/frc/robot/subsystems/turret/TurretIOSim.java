package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.constants.TurretConstants;

public class TurretIOSim implements TurretIO {

    private final SingleJointedArmSim simArm;
    private final DCMotor gearbox;
    private final ProfiledPIDController simController;

    private double appliedVolts = 0.0;
    private boolean isClosedLoop = false;
    private Rotation2d currentTargetAngle = new Rotation2d();

    public TurretIOSim() {

        gearbox = DCMotor.getNEO(1);

        double gearing = TurretConstants.kGearRatio; // 20:1

        double moi = SingleJointedArmSim.estimateMOI(
                TurretConstants.kRadius.in(Meters),
                TurretConstants.kMass.in(Kilograms));

        simArm = new SingleJointedArmSim(
                gearbox,
                gearing,
                moi,
                TurretConstants.kRadius.in(Meters),                    
                Units.rotationsToRadians(TurretConstants.kLowerLimit), 
                Units.rotationsToRadians(TurretConstants.kUpperLimit), 
                false,                                                 
                0.0);                                                  

        simController = new ProfiledPIDController(
                TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,
                new TrapezoidProfile.Constraints(2.0, 4.0)); // (maxVel, maxAccel) rot/s, rot/s²    ajustablea
    }

    @Override
    public void updateInputs(TurretInputs inputs) {

        if (isClosedLoop) {
            double measurementRot = Units.radiansToRotations(simArm.getAngleRads());
            appliedVolts = simController.calculate(measurementRot);
        }

        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

        simArm.setInputVoltage(appliedVolts);
        simArm.update(0.02);

        inputs.angle        = new Rotation2d(simArm.getAngleRads());
        inputs.targetAngle  = currentTargetAngle;
        inputs.velocityRPS  = Units.radiansToRotations(simArm.getVelocityRadPerSec());
        inputs.appliedVolts = appliedVolts;
        inputs.current      = simArm.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        isClosedLoop = false;
        appliedVolts = volts;
    }

    @Override
    public void setPosition(Rotation2d position) {
        isClosedLoop = true;
        this.currentTargetAngle = position;
        simController.setGoal(position.getRotations());
    }

    @Override
    public void setPositionWithFF(Rotation2d position, double arbFFVolts) {
        isClosedLoop = true;
        this.currentTargetAngle = position;
        simController.setGoal(position.getRotations());
        // arbFFVolts se ignora en simulación (Imadsiño)
    }

    @Override
    public void setSpeed(double dutyCycle) {
        isClosedLoop = false;
        appliedVolts = dutyCycle * TurretConstants.kMaxVolts;
    }

    @Override
    public void stop() {
        isClosedLoop = false;
        appliedVolts = 0.0;
    }

    @Override
    public void resetEnc() {
        simArm.setState(0.0, 0.0);
        simController.reset(0.0);
        currentTargetAngle = new Rotation2d();
    }
}