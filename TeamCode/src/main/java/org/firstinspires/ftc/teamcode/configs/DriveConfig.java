package org.firstinspires.ftc.teamcode.configs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
public class DriveConfig {
    public static PIDFCoefficients DRIVE_PID = new PIDFCoefficients(2, 0.3, 0.001, 11);

    /// max TPS == 2800
    public static int TPS = 2800;

    /// targetPosition 허용 오차
    public static int TOLERANCE = 1;
}
