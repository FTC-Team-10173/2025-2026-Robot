package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TestTurret extends InstantCommand {
    private final Turret turret;
    private final DoubleSupplier rotSupplier;

    public TestTurret(Turret turret, DoubleSupplier rotSupplier) {
        this.turret = turret;
        this.rotSupplier = rotSupplier;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        double position = turret.getTargetAngle();

        turret.set(position + rotSupplier.getAsDouble());
    }
}