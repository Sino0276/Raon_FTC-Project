package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmAngleManipulatorConfig {

    public static float encoder_resolution = 537.7f;

    public static double mtr_accel_min = 0.3;
    public static double mtr_decel_min = 0.1;
    public static double mtr_accel_tics = 600.0;
    public static double mtr_decel_tics = 1200.0;
    public static double tollerance = 100;
}
