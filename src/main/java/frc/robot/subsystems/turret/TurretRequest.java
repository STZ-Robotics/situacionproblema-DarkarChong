package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import com.stzteam.features.marsprocessor.CreateCommand;
import com.stzteam.features.marsprocessor.RequestFactory;
import com.stzteam.mars.diagnostics.ActionStatus;
import com.stzteam.mars.requests.Request;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.subsystems.turret.TurretIO.TurretInputs;

@RequestFactory
public interface TurretRequest extends Request<TurretInputs, TurretIO> {

    //(NO "idle"
    @CreateCommand(name = "stop")
    class Idle implements TurretRequest {
        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor) {
            actor.stop();
            data.targetAngle = data.angle;          // congela el objetivo
            return ActionStatus.of(Turret.IDLE, "Idle");
        }
    }

    @CreateCommand(name = "manualControl")
    class manualControl implements TurretRequest {
        private DoubleSupplier stick;

        public manualControl joystick(DoubleSupplier stick) {
            this.stick = stick;
            return this;
        }

        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor) {
            if (data.angle.getDegrees() < 90 && data.angle.getDegrees() > -90) {
            actor.setSpeed(stick.getAsDouble() * 0.5);   
            } else {
            actor.setSpeed(0);                          
            }
            return ActionStatus.of(Turret.MANUAL_CONTROL, "Manual");
        }
    }

    @CreateCommand(name = "toPosition")
    class Position implements TurretRequest {
        private Rotation2d m_targetAngle = new Rotation2d();
        private double toleranceDegrees = 1.0;

        public Position withTargetAngle(Rotation2d angle) {
            this.m_targetAngle = angle;
            return this;
        }

        public Position withTolerance(double toleranceDegrees) {
            this.toleranceDegrees = toleranceDegrees;
            return this;
        }

        @Override
        public ActionStatus apply(TurretInputs data, TurretIO actor) {
            data.targetAngle = m_targetAngle;
            actor.setPosition(m_targetAngle);

            boolean isLocked =
                MathUtil.isNear(m_targetAngle.getDegrees(), data.angle.getDegrees(), toleranceDegrees);

            return isLocked
                ? ActionStatus.of(Turret.LOCKED, "Objetivo alcanzado")
                : ActionStatus.of(Turret.TRACKING,
                      "Siguiendo a " + Math.round(m_targetAngle.getDegrees()) + "°");
        }
    }
}