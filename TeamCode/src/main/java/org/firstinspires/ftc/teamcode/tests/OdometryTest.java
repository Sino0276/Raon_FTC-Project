package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "OdometryTest", group = "java")
public class OdometryTest extends LinearOpMode {

    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.initialize();
        odo.setOffsets(104.6, -72, DistanceUnit.MM);  // 임시설정 - 수정할것
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        waitForStart();
        if (opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            odo.resetPosAndIMU();

            while (opModeIsActive()) {
                odo.update();
                packet.put("Encoder_X", odo.getEncoderX());
                packet.put("Encoder_Y", odo.getEncoderY());
                packet.put("Pos_X", odo.getPosX(DistanceUnit.MM));
                packet.put("Pos_Y", odo.getPosY(DistanceUnit.MM));
                packet.put("Velocity_X", odo.getVelX(DistanceUnit.MM));
                packet.put("Velocity_Y", odo.getVelY(DistanceUnit.MM));
                packet.put("Degree", odo.getHeading(AngleUnit.DEGREES));

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}
