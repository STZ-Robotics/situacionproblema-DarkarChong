// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.stzteam.mars.models.containers.IRobotContainer;
import com.stzteam.mars.operator.ControllerOI;
import com.stzteam.mars.test.TestRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.bindings.OperatorBindings;
import frc.robot.configuration.Manifest;
import frc.robot.subsystems.turret.Turret;
import frc.tests.EmptyTest;
import frc.tests.TurretTest;

public class RobotContainer implements IRobotContainer {

    public final Turret turret;
    public final ControllerOI operator;

    public RobotContainer() {
        this.turret   = Manifest.buildTurret();
        this.operator = Manifest.ControlsBuilder.buildOperator();

        OperatorBindings.create(operator, turret).bind();
        
    }

    @Override
    public void updateNodes() {}

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    @Override
    public TestRoutine getTestRoutine() {
        return new TurretTest(turret);
    }
}