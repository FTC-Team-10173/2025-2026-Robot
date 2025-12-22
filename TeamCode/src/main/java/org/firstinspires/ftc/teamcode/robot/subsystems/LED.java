package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LED {
    GoBildaPrismDriver prism;
    Servo indicator;

    // LED animations
    PrismAnimations.Solid idle = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid ready = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Blink spinUp = new PrismAnimations.Blink(Color.PINK);

    public boolean shooterReady = false; // true if flywheel is at target speed
    public boolean spinningUp = false; // true if flywheel is spinning up

    // constructor
    public LED(HardwareMap hardwareMap) {
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
    }

    // periodic method to be called in main loop
    public void periodic() {
        /*
         * 0 - (idle) solid hot pink
         * 1 - (flywheel spin up) slow flash hot pink
         * 2 - (flywheel at target speed) fast flash hot pink
         */
        if (shooterReady) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, ready);
            indicator.setPosition(0.500);
        } else if (spinningUp) {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, spinUp);
            indicator.setPosition(0.388);
        } else {
            prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, idle);
            indicator.setPosition(0.722);
        }
    }
}
