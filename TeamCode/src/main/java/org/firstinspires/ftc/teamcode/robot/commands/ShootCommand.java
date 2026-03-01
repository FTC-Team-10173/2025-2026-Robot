package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;

import java.util.function.BiFunction;
import java.util.function.Supplier;

public class ShootCommand extends CommandBase {
    private final Shooter shooter;
    private final Intake intake;
    private final Supplier<Pose2d> poseSupplier;
    private final Constants.Alliance alliance;
    private final BiFunction<Pose2d, Constants.Alliance, Double> powerSupplier;

    public ShootCommand(
            Shooter shooter,
            Intake intake,
            Supplier<Pose2d> poseSupplier,
            Constants.Alliance alliance,
            BiFunction<Pose2d, Constants.Alliance, Double> powerSupplier
    ) {
        this.shooter = shooter;
        this.intake = intake;
        this.poseSupplier = poseSupplier;
        this.alliance = alliance;
        this.powerSupplier = powerSupplier;

        addRequirements(shooter, intake);
    }

    @Override
    public void initialize() {
        shooter.setPower(
                powerSupplier.apply(poseSupplier.get(), alliance)
        );
        shooter.startFlywheel();
    }

    @Override
    public void execute() {
        // Update power if needed
        shooter.setPower(
                powerSupplier.apply(poseSupplier.get(), alliance)
        );
        shooter.feed(
                intake::fullIntake,
                intake::openGate
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFlywheel();
    }
}