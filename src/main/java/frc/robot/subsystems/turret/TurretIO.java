package frc.robot.subsystems.turret;

import com.stzteam.features.marsprocessor.Fallback;
import com.stzteam.mars.models.singlemodule.Data;
import com.stzteam.mars.models.singlemodule.IO;
import edu.wpi.first.math.geometry.Rotation2d;

@Fallback
public interface TurretIO extends IO<TurretIO.TurretInputs> {

    class TurretInputs extends Data<TurretInputs> {
        public Rotation2d angle = new Rotation2d();
        public Rotation2d targetAngle = new Rotation2d();
        public double velocityRPS = 0.0;
        public double appliedVolts = 0.0;
        public double current = 0.0;

        
    }

    default void updateInputs(TurretInputs inputs) {}

    default void setVoltage(double volts) {}

    default void setPosition(Rotation2d position) {}

    default void setPositionWithFF(Rotation2d position, double arbFFVolts) {}

    default void setSpeed(double dutyCycle) {}

    default void stop() {}

    default void resetEnc() {}
}
