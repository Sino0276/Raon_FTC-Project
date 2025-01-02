package org.firstinspires.ftc.teamcode;

public class Utilities {

    static double COUNTS_PER_MOTOR_REV = 537;
    static double DRIVE_GEAR_REDUCTION = 1;
    static double WHEEL_DIAMETER_INCHES = 3.7;
    static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static double angleWrap(double wrappingAngle) {
        while (wrappingAngle > 180) {
            wrappingAngle = wrappingAngle - 360;
        }
        while (wrappingAngle < -180) {
            wrappingAngle = wrappingAngle + 360;
        }
        return wrappingAngle;
    }

    public static int inchToEncoder(double inch) { return (int)(inch * COUNTS_PER_INCH); }
}
