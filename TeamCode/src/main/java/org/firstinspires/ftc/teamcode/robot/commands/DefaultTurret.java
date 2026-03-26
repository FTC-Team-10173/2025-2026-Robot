package org.firstinspires.ftc.teamcode.robot.commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class DefaultTurret extends CommandBase {
    private final Turret turret;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier headingSupplier;
    private final Function<Pose2d, Double> targetSupplier;

    public DefaultTurret(
            Turret turret,
            Function<Pose2d, Double> targetSupplier,
            Supplier<Pose2d> poseSupplier,
            DoubleSupplier headingSupplier
    ) {
        this.turret = turret;
        this.poseSupplier = poseSupplier;
        this.headingSupplier = headingSupplier;
        this.targetSupplier = targetSupplier;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d pose = poseSupplier.get();
        double correctedHeading = headingSupplier.getAsDouble();

        double targetHeading = targetSupplier.apply(
                new Pose2d(
                        pose.position.x, pose.position.y, correctedHeading
                )
        );

        turret.set(targetHeading);
    }
}