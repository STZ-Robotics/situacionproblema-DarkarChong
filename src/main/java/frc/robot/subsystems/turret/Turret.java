package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import com.stzteam.forgemini.io.NetworkIO;
import com.stzteam.mars.diagnostics.ModuleColorCode;
import com.stzteam.mars.diagnostics.StatusColorCode.Severity;
import com.stzteam.mars.models.SubsystemBuilder;
import com.stzteam.mars.models.Telemetry;
import com.stzteam.mars.models.singlemodule.ModularSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.configuration.KeyManager;
import frc.robot.subsystems.turret.TurretIO.TurretInputs;


public class Turret extends ModularSubsystem<TurretInputs, TurretIO>
        implements TurretCommands {                 

    public static final ModuleColorCode IDLE =
        ModuleColorCode.solid("IDLE", Severity.OK, Color.kDarkGreen, "Torreta en reposo");
    public static final ModuleColorCode LOCKED =
        ModuleColorCode.solid("LOCKED", Severity.OK, Color.kFirstBlue, "Torreta bloqueada");
    public static final ModuleColorCode TRACKING =
        ModuleColorCode.solid("TRACKING", Severity.WARNING, Color.kYellow, "Torreta en seguimiento");
    public static final ModuleColorCode MANUAL_CONTROL =
        ModuleColorCode.solid("MANUAL_CONTROL", Severity.WARNING, Color.kBrown, "Control manual");

    public Turret(TurretIO io) {
        super(SubsystemBuilder.<TurretInputs, TurretIO>setup()
            .key(KeyManager.TURRET_KEY)
            .hardware(io, new TurretInputs())
            .request(new TurretRequest.Idle())
            .telemetry(new TurretTelemetry()));

        setDefaultCommand(runRequest(() -> new TurretRequest.Idle()));
    }

    public boolean isAtTarget(double toleranceDegrees) {
        return MathUtil.isNear(
            inputs.targetAngle.getDegrees(),
            inputs.angle.getDegrees(),
            toleranceDegrees);
    }

    @Override
    public TurretInputs getState() {
        return inputs;
    }

    @Override
    public Command setControl(Supplier<TurretRequest> request) {
        return runRequest(request);
    }

    @Override
    public void absolutePeriodic(TurretInputs data) {}

    @Override
    public void simulationPeriodic() {}

    static class TurretTelemetry extends Telemetry<TurretInputs> {
        @Override
        public void telemeterize(TurretInputs d) {
            NetworkIO.set(KeyManager.TURRET_KEY, "Voltage",  d.appliedVolts);
            NetworkIO.set(KeyManager.TURRET_KEY, "Angle",    d.angle.getDegrees());
            NetworkIO.set(KeyManager.TURRET_KEY, "Target",   d.targetAngle.getDegrees());
            NetworkIO.set(KeyManager.TURRET_KEY, "Velocity", d.velocityRPS);
            NetworkIO.set(KeyManager.TURRET_KEY, "Latency",  d.timestamp);
        }
    }
}