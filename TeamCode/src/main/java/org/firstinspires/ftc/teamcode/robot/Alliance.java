package org.firstinspires.ftc.teamcode.robot;

public class Alliance {
    private static boolean blue = true;

    public static boolean isBlue() {
        return blue;
    }

    public static boolean isRed() {
        return !blue;
    }

    public static void setBlue() {
        blue = true;
    }

    public static void setRed() {
        blue = false;
    }
}
