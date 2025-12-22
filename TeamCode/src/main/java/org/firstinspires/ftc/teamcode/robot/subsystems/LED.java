package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class LED {
    GoBildaPrismDriver prism;
    Servo indicator;
    RobotState robotState;

    // LED animations
    PrismAnimations.Solid idle = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid ready = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Blink spinUp = new PrismAnimations.Blink(Color.PINK);

    // constructor
    public LED(HardwareMap hardwareMap, RobotState robotState) {
        // initialize servo and prism driver
        indicator = hardwareMap.get(Servo.class, "indicator");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        // set prism strip length, 36 LEDs
        prism.setStripLength(36);

        // setting configurations for animations
        // idle animation settings
        idle.setBrightness(100);
        idle.setStartIndex(0);
        idle.setStopIndex(36);

        // ready animation settings
        ready.setBrightness(100);
        ready.setStartIndex(0);
        ready.setStopIndex(36);

        // spin up animation settings
        spinUp.setBrightness(100);
        spinUp.setStartIndex(0);
        spinUp.setStopIndex(36);
        spinUp.setPeriod(1000); // 1 second period

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
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, ready);
            indicator.setPosition(0.500);
        } else if (robotState.is(RobotState.State.SHOOTING)) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, spinUp);
            indicator.setPosition(0.388);
        } else {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, idle);
            indicator.setPosition(0.722);
        }
    }
}
