package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class ShooterConfig {
    public static PIDFCoefficients TURNING_PID = new PIDFCoefficients(2, 0.3, 0.001, 11);

    public static int TPS = 2800;
}
