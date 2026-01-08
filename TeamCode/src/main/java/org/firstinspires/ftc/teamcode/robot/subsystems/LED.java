package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class LED implements Subsystem {
    GoBildaPrismDriver prism;
    Servo indicator;
    RobotState robotState;

    // LED animations
    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.PINK);
    // constructor
    public LED(HardwareMap hardwareMap, RobotState robotState) {
        // initialize servo and prism driver
        indicator = hardwareMap.get(Servo.class, "indicator");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // set prism strip length, 36 LEDs
        prism.setStripLength(36);

        // solid animation settings
        solid.setBrightness(100);
        solid.setStartIndex(0);
        solid.setStopIndex(36);

        // insert solid animation into layer 0
        prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solid);

        // store robot state
        this.robotState = robotState;
    }

    // periodic method to be called in main loop
    public void periodic() {
        /*
         * 0 - (idle) solid hot pink
         * 1 - (flywheel spin up) slow flash hot pink
         * 2 - (flywheel at target speed) fast flash hot pink
         */
        if (robotState.is(RobotState.State.SHOOTING_READY)) {
            indicator.setPosition(0.500);
        } else if (robotState.is(RobotState.State.SPINNING_UP)) {
            indicator.setPosition(0.388);
        } else {
            indicator.setPosition(0.722);
        }
    }

    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

    public boolean isHealthy() {
        return prism != null && indicator != null;
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addData(getName() + " LED count", "%d", prism.getNumberOfLEDs());
        telemetry.addData(getName() + " FPS", "%d", prism.getCurrentFPS());
        telemetry.addData(getName() + " Indicator Position", "%.3f", indicator.getPosition());
        telemetry.addData(getName() + " Healthy", isHealthy());
    }
}
