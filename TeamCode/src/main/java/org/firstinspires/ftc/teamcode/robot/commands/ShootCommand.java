package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import java.util.function.DoubleSupplier;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier powerSupplier;

    public ShootCommand(Shooter shooter, DoubleSupplier powerSupplier) {
        this.shooter = shooter;
        this.powerSupplier = powerSupplier;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setPower(powerSupplier.getAsDouble());
        shooter.startFlywheel();
    }

    @Override
    public void execute() {
        // Update power if needed
        shooter.setPower(powerSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}