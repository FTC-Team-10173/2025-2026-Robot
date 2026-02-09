package org.firstinspires.ftc.teamcode.robot.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.function.BiFunction;
import java.util.function.Supplier;

public class DefaultTurret extends CommandBase {
    private final Turret turret;
    private final Constants.Alliance alliance;
    private final Supplier<Pose2d> poseSupplier;
    private final BiFunction<Pose2d, Constants.Alliance, Double> targetSupplier;

    public DefaultTurret(Turret turret, Constants.Alliance alliance, BiFunction<Pose2d, Constants.Alliance, Double> targetSupplier, Supplier<Pose2d> poseSupplier) {
        this.turret = turret;
        this.alliance = alliance;
        this.poseSupplier = poseSupplier;
        this.targetSupplier = targetSupplier;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        double targetHeading = targetSupplier.apply(poseSupplier.get(), alliance);

        turret.set(targetHeading);
    }
}