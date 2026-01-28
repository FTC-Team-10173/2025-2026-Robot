package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class PowerChange extends InstantCommand {
    Shooter shooter;
    DoubleSupplier powerSupplier;
    public PowerChange(Shooter shooter, DoubleSupplier powerSupplier) {
        this.shooter = shooter;
        this.powerSupplier = powerSupplier;
    }

    @Override
    public void execute() {
        double power = powerSupplier.getAsDouble();

        shooter.changeTestPower(power);
    }
}
