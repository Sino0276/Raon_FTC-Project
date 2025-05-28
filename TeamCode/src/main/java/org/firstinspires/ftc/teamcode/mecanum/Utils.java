package org.firstinspires.ftc.teamcode.mecanum;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Utils /*implements IMecanum*/ {

    private LinearOpMode linearOpMode;
    private FtcDashboard dashboard;

    public Utils(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        this.linearOpMode = linearOpMode;
        this.dashboard = dashboard;
    }

    public void addData(String caption, String value) {
        linearOpMode.telemetry.addData(caption, value);
        dashboard.getTelemetry().addData(caption, value);
    }

    public void update() {
        linearOpMode.telemetry.update();
        dashboard.getTelemetry().update();
    }

}
