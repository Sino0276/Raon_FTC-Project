package org.firstinspires.ftc.teamcode.mecanum.traction;

import com.acmerobotics.dashboard.config.Config;

@Config
public class MecanumTractionConfig {

    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended forward-backward motion when sideways motion is requested.
    public static double V_forward_V_side_bias = 0.001;
    // After calibration this term represents the bias of the mecanum drive as a result of alignment, friction, etc.
    // that results in un-intended turn motion when sideways motion is requested.
    public static double V_turn_V_side_bias = 0.001;

    // The constants that regulate this program - adjust these to your physical
    // implementation of the drive.
    public static double mtr_accel_min = 0.3;
    public static double mtr_decel_min = 0.1;
    public static double mtr_accel_tics = 600.0;
    public static double mtr_decel_tics = 1200.0;
    public static double mtr_accel_degs = 20.0;
    public static double mtr_decel_degs = 30.0;
    public static double tics_per_inch_forward = 84.0;
    public static double tics_per_inch_sideways = 83.0;
    // A turning rate when in automotive drive mode to limit the turn rate at full
    // forward or backward speed.
    public static double auto_turn_rate = 0.15;
}
