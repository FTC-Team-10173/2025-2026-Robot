package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import static org.firstinspires.ftc.teamcode.Prism.Color.*;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.robot.Logger;

public class LED extends SubsystemBase {
    private final GoBildaPrismDriver prism;
    private final Servo indicator;
    private State currentState = State.IDLE;

    public LED(HardwareMap hardwareMap) {
        indicator = hardwareMap.get(Servo.class, "indicator");
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        prism.setStripLength(36);

        // Setup default animation
        PrismAnimations.Solid solid = new PrismAnimations.Solid(PINK);
        solid.setBrightness(100);
        solid.setStartIndex(0);
        solid.setStopIndex(36);

        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        indicator.setPosition(0.722); // Idle position
    }

    public void setRobotState(State state) {
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
                indicator.setPosition(PWMColor.GREEN.getPMW());
                break;
            case SPINNING_UP:
                indicator.setPosition(PWMColor.YELLOW.getPMW());
                break;
            case IDLE:
            default:
                indicator.setPosition(PWMColor.PINK.getPMW());
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

    public void set(State state) {
        currentState = state;
    }

    public enum State {
        IDLE,
        SHOOTING_READY,
        SPINNING_UP
    }

    public enum PWMColor {
        PINK(0.722),
        YELLOW(0.388),
        GREEN(0.500);

        private final double pmw;

        public double getPMW() { return pmw; }

        PWMColor(double pmw) {
            this.pmw = pmw;
        }
    }
}