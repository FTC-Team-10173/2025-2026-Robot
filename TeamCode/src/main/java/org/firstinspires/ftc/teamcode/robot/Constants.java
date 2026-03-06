package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import java.util.HashMap;


public final class Constants {
    
    public static class Drive {
        // PID constants for heading lock
        public static final double HEADING_KP = 0.65;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.0;
        public static final double DEADZONE = 0.1;
    }

    
    public static class StartingPoses {
        public static Pose2d BLUE_CLOSE = new Pose2d(-54, -48, Math.toRadians(225));
        public static Pose2d BLUE_FAR = new Pose2d(64, -16, Math.toRadians(180));
        public static Pose2d RED_CLOSE = new Pose2d(-54, 48, Math.toRadians(135));
        public static Pose2d RED_FAR = new Pose2d(64, 16, Math.toRadians(180));
    }

    
    public static class ShootingPoses {
        public static final Pose2d BLUE_CLOSE = new Pose2d(-18, -18, Math.toRadians(225)); // 46.86149806
        public static Pose2d BLUE_CLOSE_BACK = new Pose2d(-18, -18, Math.toRadians(-45));
        public static final Pose2d RED_CLOSE = new Pose2d(-18, 18, Math.toRadians(135)); // 0.001368206908*46.86149806
        public static Pose2d RED_CLOSE_BACK = new Pose2d(-18, 18, Math.toRadians(45));
        public static final Pose2d BLUE_FAR = new Pose2d(58, -16, Math.toRadians(200)); // 112.6410227
        public static final Pose2d RED_FAR = new Pose2d(58, 16, Math.toRadians(160)); // 112.6410227
    }

    
    public static class GoalPoses {
        public static final Translation2d BLUE = new Translation2d(-72, -72);
        public static final Translation2d RED = new Translation2d(-72, 72);

        public static Translation2d get(Alliance alliance) {
            return alliance == Alliance.BLUE ? BLUE : RED;
        }

        public static Translation2d getTurretGoal(Alliance alliance) {
            if (alliance == Alliance.BLUE) {
                return new Translation2d(
                        BLUE.getX() + 12, BLUE.getY() + 12
                );
            } else {
                return new Translation2d(
                        RED.getX() + 12, RED.getY() - 12
                );
            }
        }
    }

    public static class Tags {
        public static int BLUE = 20;
        public static int RED = 24;

        public static int getID(Alliance alliance) {
            return alliance == Alliance.BLUE ? BLUE : RED ;
        }
    }
    
    public static class ShootingPower { // 0.001368206908
        public static final double CLOSE = 0.375;
        public static final double FAR = 0.47;
    }

    
    public static class ParkingPoses {
        public static final Pose2d BLUE_CLOSE = new Pose2d(-24, -48, Math.toRadians(270));
        public static final Pose2d RED_CLOSE = new Pose2d(-24, 48, Math.toRadians(90));
        public static final Pose2d BLUE_FAR = new Pose2d(60, -36, Math.toRadians(270));
        public static final Pose2d RED_FAR = new Pose2d(60, 36, Math.toRadians(90));
    }

    
    public static class GatePoses {
        public static final Pose2d BLUE_OPEN = new Pose2d(0, -54, Math.toRadians(270));
        public static final Pose2d BLUE_INTAKE = new Pose2d(9, -66, Math.toRadians(225));
        public static final Pose2d RED_OPEN = new Pose2d(0, 54, Math.toRadians(90));
        public static final Pose2d RED_INTAKE = new Pose2d(9, 66, Math.toRadians(135));

    }

    
    public static class Gate {
        public static final double MIN_ANGLE = 0;
        public static final double MAX_ANGLE = 1800;
        public static double OPEN_ANGLE = 1218;
        public static double CLOSED_ANGLE = 1800;

        static {
            validateAngles();
        }

        private static void validateAngles() {
            if (OPEN_ANGLE < MIN_ANGLE || OPEN_ANGLE > MAX_ANGLE) {
                throw new IllegalStateException("OPEN_ANGLE out of range");
            }
            if (CLOSED_ANGLE < MIN_ANGLE || CLOSED_ANGLE > MAX_ANGLE) {
                throw new IllegalStateException("CLOSED_ANGLE out of range");
            }
        }
    }

    
    public static class Turret {
        public static final double RANGE_MIN_ANGLE = 0;
        public static final double RANGE_MAX_ANGLE = 1800;
        public static final double RANGE = 90;
        public static final double GEAR_RATIO = (double) 150/50;
    }

    
    public static class Shooter {
        // Shooter velocity control constants
        public static final double BASE = 1.0054;
        public static final double INTERCEPT = 0.258;
        public static final double VELOCITY_TOLERANCE = 40;
        public static final double MAX_RPM = 2800;

        // Feedforward gains
        public static final double kP = 5.0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.5;
        public static final double kA = 0;
    }

    
    public static class Intake {
        public static final double FEED_TIME_SEC = 1.25;
        public static final double GATE_TIME_SEC = 1.25;
    }

    
    public static class Vision {
        // Camera settings
        public static final int EXPOSURE = 25;
        public static final int GAIN = -325;

        // AprilTag IDs
        public static final int BLUE_GOAL_ID = 20;
        public static final int RED_GOAL_ID = 24;
    }

    public static HashMap<String, Object> BlackBoard = new HashMap<>();

    
    public static class Keys {
        public static final String POSE = "pose";
        public static final String ALLIANCE = "alliance";
    }

    
    public enum Alliance { RED, BLUE }
}