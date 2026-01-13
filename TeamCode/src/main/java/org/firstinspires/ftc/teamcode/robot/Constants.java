package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public final class Constants {
    public static class Drive {
        // PID constants for heading lock
        public static final double HEADING_KP = 0.02;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.0;
        public static final double DEADZONE = 0.1;
    }

    public static class StartingPoses {
        public static Pose2d BLUE_CLOSE = new Pose2d(-54, -48, Math.toRadians(225));
        public static Pose2d BLUE_FAR = new Pose2d(-54, -48, Math.toRadians(225));
        public static Pose2d RED_CLOSE = new Pose2d(-54, 48, Math.toRadians(135));
        public static Pose2d RED_FAR = new Pose2d(-54, 48, Math.toRadians(135));
    }

    public static class ShootingPoses {
        public static final Pose2d BLUE_CLOSE = new Pose2d(-18, -18, Math.toRadians(225));
        public static final Pose2d RED_CLOSE = new Pose2d(-18, 18, Math.toRadians(135));
        public static final Pose2d BLUE_FAR = new Pose2d(54, -16, Math.toRadians(204));
        public static final Pose2d RED_FAR = new Pose2d(54, 16, Math.toRadians(156));
    }

    public static class ShootingPower {
        public static final double CLOSE = 0.40;
        public static final double FAR = 0.475;
    }

    public static class ParkingPoses {
        public static final Pose2d BLUE_CLOSE = new Pose2d(-60, -12, Math.toRadians(90));
        public static final Pose2d RED_CLOSE = new Pose2d(-60, 12, Math.toRadians(270));
        public static final Pose2d BLUE_FAR = new Pose2d(60, -36, Math.toRadians(90));
        public static final Pose2d RED_FAR = new Pose2d(60, 36, Math.toRadians(270));
    }

    public static class GatePoses {
        public static final Pose2d BLUE_OPEN = new Pose2d(0, -54, Math.toRadians(270));
        public static final Pose2d BLUE_INTAKE = new Pose2d(6, -60, Math.toRadians(225));
        public static final Pose2d RED_OPEN = new Pose2d(0, 54, Math.toRadians(90));
        public static final Pose2d RED_INTAKE = new Pose2d(6, 60, Math.toRadians(135));

    }

    public static class Gate {
        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 1800;
        public static double OPEN_ANGLE = 1494;
        public static double CLOSED_ANGLE = 1674;

        static {
            validateAngles();
        }

        private static void validateAngles() {
            if (OPEN_ANGLE < MIN_ANGLE || OPEN_ANGLE > MAX_ANGLE) {
                throw new IllegalStateException("OPEN_ANGLE out of range");
            }
            if (CLOSED_ANGLE < MIN_ANGLE || CLOSED_ANGLE > MAX_ANGLE) {
                throw new IllegalStateException("OPEN_ANGLE out of range");
            }
        }
    }

    public static class Hood {
        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 1;
    }

    public static class Shooter {
        // Shooter velocity control constants
        public static final double SCALE = 1000;
        public static final double INTERCEPT = 120;
        public static final double MAX_RPM = 2400;
        public static final double VELOCITY_TOLERANCE = 20;

        // Feedforward gains
        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.7;
        public static final double kA = 0;
    }

    public static class Intake {
        public static final double INTAKE_POWER = 1.0;
        public static final double OUTTAKE_POWER = -0.6;
        public static final double FEED_TIME_SEC = 3.0;
    }

    public static class Vision {
        // Camera settings
        public static final int EXPOSURE = 25;
        public static final int GAIN = -325;

        // AprilTag IDs
        public static final int BLUE_GOAL_ID = 20;
        public static final int RED_GOAL_ID = 24;
    }
}