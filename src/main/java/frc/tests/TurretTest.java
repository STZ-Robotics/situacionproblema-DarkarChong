package frc.tests;

import com.stzteam.mars.test.MARSTest;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequestFactory;   

@MARSTest(name = "Turret Test")
public class TurretTest extends TestRoutine {

    private final Turret t;

    public TurretTest(Turret turret) {
        this.t = turret;
    }

    @Override
    public Command getRoutineCommand() {
        return Commands.sequence(
            //  Ir a 45°
            run(TurretRequestFactory.position()
                    .withTargetAngle(Rotation2d.fromDegrees(45))
                    .withTolerance(2), t),
            waitFor(() -> t.isAtTarget(2), 3),
            assertLessThan(
                calculateError(Units.degreesToRadians(45), t.getState().angle.getRadians()),
                2.0, "High turret error on target 1"),
            delay(1),

            // Ir a -45°
            run(TurretRequestFactory.position()
                    .withTargetAngle(Rotation2d.fromDegrees(-45))
                    .withTolerance(2), t),
            waitFor(() -> t.isAtTarget(2), 3),
            assertLessThan(
                calculateError(Units.degreesToRadians(-45), t.getState().angle.getRadians()),
                2.0, "High turret error on target 2"),
            delay(1),

            run(TurretRequestFactory.position()
                    .withTargetAngle(new Rotation2d())   // 0°
                    .withTolerance(2), t),
            waitFor(() -> t.isAtTarget(2), 3),
            assertLessThan(
                calculateError(0.0, t.getState().angle.getRadians()),
                2.0, "High turret error on target 3"),

            run(TurretRequestFactory.idle(), t)
        );
    }
}