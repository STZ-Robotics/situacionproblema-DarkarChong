package frc.robot.bindings;

import com.stzteam.mars.models.containers.Binding;
import com.stzteam.mars.operator.ControllerOI;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretRequestFactory;   // autogenerada al hacer Build

public class OperatorBindings implements Binding {

    private final ControllerOI operator;
    private final Turret turret;

    private OperatorBindings(ControllerOI operator, Turret turret) {
        this.operator = operator;
        this.turret = turret;
    }

    public static OperatorBindings create(ControllerOI operator, Turret turret) {
        return new OperatorBindings(operator, turret);
    }

    @Override
    public void bind() {
        var leftStick = operator.getLeftStick();   
        var pov       = operator.getDPadTriggers(); 

        
        Trigger leftStickXTrigger =
            new Trigger(() -> Math.abs(leftStick.x().getAsDouble()) > 0.1);

        // Stick X movido  Y  D-Pad derecha presionada (al mismo tiempo) 
        leftStickXTrigger.and(pov.right()).whileTrue(
            turret.setControl(() ->
                TurretRequestFactory.manualControl().joystick(leftStick.x())));
    }
}