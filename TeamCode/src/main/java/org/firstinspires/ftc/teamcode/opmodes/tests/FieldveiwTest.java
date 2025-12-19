package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;

@Disabled
@TeleOp(name = "FieldveiwTest", group = "Test")
public class FieldveiwTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        waitForStart();
        if (opModeIsActive()) {

            while (opModeIsActive()) {

                packet.fieldOverlay().setFill("blue").fillRect(-20, -20, 40, 40);
                dashboard.sendTelemetryPacket(packet);

            }
        }
    }
}
