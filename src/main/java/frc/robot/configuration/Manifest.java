package frc.robot.configuration;

import com.stzteam.mars.builder.Environment;
import com.stzteam.mars.builder.Environment.RunMode;
import com.stzteam.mars.builder.Injector;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.operator.PS5OI;
import com.stzteam.mars.operator.XboxOI;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOFallback;   
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOSparkMax;

public class Manifest {

    //cambbiar a RunMode.REAL para el robot físico
    public static final RunMode CURRENT_MODE = RunMode.SIM;
    static { Environment.setMode(CURRENT_MODE); }

    // si fuera false, el Injector usa el Fallback 
    public static final boolean HAS_TURRET = true;

    
    public enum ControllerType { PS5, XBOX }
    public static final ControllerType OPERATOR_CONTROLLER = ControllerType.XBOX;
    private static final int OPERATOR_PORT = 1;

    public static Turret buildTurret() {
        TurretIO io = Injector.createIO(
            HAS_TURRET,
            TurretIOFallback::new,   // HAS_TURRET == false
            TurretIOSparkMax::new,   // RunMode.REAL
            TurretIOSim::new);       // RunMode.SIM
        return new Turret(io);
    }

    public static class ControlsBuilder {
        public static ControllerOI buildOperator() {
            return OPERATOR_CONTROLLER == ControllerType.PS5
                ? new PS5OI(OPERATOR_PORT)
                : new XboxOI(OPERATOR_PORT);
        }
    }
}