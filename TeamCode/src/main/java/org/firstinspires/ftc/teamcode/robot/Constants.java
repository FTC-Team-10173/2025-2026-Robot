package org.firstinspires.ftc.teamcode.robot;

public class Constants {
    public static class Drive {
        public static final double HEADING_KP = 0.02;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.0;
    }

    public static class Shooter {
        public static final double SCALE = 670;
        public static final double INTERCEPT = 85;
        public static final double MAX_RPM = 2400;
        public static final double VELOCITY_TOLERANCE = 20;

        public static final double kP = 20;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0;
        public static final double kV = 0.7;
        public static final double kA = 0;
    }

    public static class Vision {
        public static final int EXPOSURE = 25;
        public static final int GAIN = -325;
        public static final int BLUE_GOAL_ID = 20;
        public static final int RED_GOAL_ID = 24;
    }
}