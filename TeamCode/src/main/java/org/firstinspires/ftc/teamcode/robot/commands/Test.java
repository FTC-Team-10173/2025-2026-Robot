package org.firstinspires.ftc.teamcode.robot.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Test extends CommandBase {
    private final Shooter shooter;
    private final BooleanSupplier isPositive;

    public Test(Shooter shooter, BooleanSupplier isPositive) {
        this.shooter = shooter;
        this.isPositive = isPositive;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.test(isPositive.getAsBoolean());
    }
}