package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.robot.Logger;
import org.firstinspires.ftc.teamcode.robot.RobotState;

public class LED extends SubsystemBase {
    private final GoBildaPrismDriver prism;
    private final Servo indicator;
    private RobotState.State currentState = RobotState.State.IDLE;

    public LED(HardwareMap hardwareMap) {
        indicator = hardwareMap.get(Servo.class, "indicator");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        prism.setStripLength(36);

        // Setup default animation
        PrismAnimations.Solid solid = new PrismAnimations.Solid(org.firstinspires.ftc.teamcode.Prism.Color.PINK);
        solid.setBrightness(100);
        solid.setStartIndex(0);
        solid.setStopIndex(36);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        indicator.setPosition(0.722); // Idle position
    }

    public void setRobotState(RobotState.State state) {
        this.currentState = state;
        updateIndicator();
    }

    @Override
    public void periodic() {
        updateIndicator();
    }

    private void updateIndicator() {
        switch (currentState) {
            case SHOOTING_READY:
                indicator.setPosition(0.500);
                break;
            case SPINNING_UP:
                indicator.setPosition(0.388);
                break;
            case IDLE:
            default:
                indicator.setPosition(0.722);
                break;
        }
    }

    public boolean isHealthy() {
        return prism != null && indicator != null;
    }

    public void stop() {
        prism.clearAllAnimations();
        prism.updateAllAnimations();
    }

    public void updateTelemetry(Telemetry telemetry, Logger logger) {
        telemetry.addData(getName() + " Healthy", isHealthy());
        telemetry.addData(getName() + " State", currentState);

        if (logger != null) {
            logger.put(getName() + " Healthy", isHealthy());
            logger.put(getName() + " State", currentState.toString());
        }
    }
}