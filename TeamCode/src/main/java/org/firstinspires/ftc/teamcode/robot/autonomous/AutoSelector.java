package org.firstinspires.ftc.teamcode.robot.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.robot.autonomous.paths.BlueCloseFull;
import org.firstinspires.ftc.teamcode.robot.autonomous.paths.RedCloseFull;

import com.pedropathing.geometry.Pose;

public final class AutoSelector {

    public static final String GROUP = "Autonomous";
    public static final boolean DISABLED = false;

    private AutoSelector() {}

    private static OpModeMeta meta(String name) {
        return new OpModeMeta.Builder()
                .setName(name)
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.AUTONOMOUS)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        manager.register(meta("Blue Close Full"), new AutoBase(
                new Pose(25, 127, Math.toRadians(135)),
                BlueCloseFull::new
        ));

        manager.register(meta("Red Close Full"), new AutoBase(
                new Pose(119, 127, Math.toRadians(45)),
                RedCloseFull::new
        ));
    }
}