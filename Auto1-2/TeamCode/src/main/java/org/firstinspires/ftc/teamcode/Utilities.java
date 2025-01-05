package org.firstinspires.ftc.teamcode;

public class Utilities {

    private static final double COUNTS_PER_MOTOR_REV = 537;
    private static final double DRIVE_GEAR_REDUCTION = 1;
    private static final double WHEEL_DIAMETER_INCHES = 3.7;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    static final int armAngleRev = 13425;    // 팔이 한바퀴 돌기 위해 필요한 인코더 값

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
