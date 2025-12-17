package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Disabled
@TeleOp(name = "OdometryTest", group = "Test")
public class OdometryTest extends LinearOpMode {
    private GoBildaPinpointDriver odo;


    /// Tick Per Inch = 505.33
    @Override
    public void runOpMode() {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.initialize();
//        odo.setOffsets(3, -72, DistanceUnit.MM);
        odo.setOffsets(-5, -62.5, DistanceUnit.MM);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        waitForStart();
        if (opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            odo.resetPosAndIMU();
            odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));

            while (opModeIsActive()) {

                if (gamepad1.a) {
                    odo.resetPosAndIMU();
                    odo.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
                }

                odo.update();
                packet.put("Encoder_X", odo.getEncoderX());
                packet.put("Encoder_Y", odo.getEncoderY());
                packet.put("Pos_X", odo.getPosX(DistanceUnit.INCH));
                packet.put("Pos_Y", odo.getPosY(DistanceUnit.INCH));
//                packet.put("Velocity_X", odo.getVelX(DistanceUnit.INCH));
//                packet.put("Velocity_Y", odo.getVelY(DistanceUnit.INCH));
                packet.put("Degree", odo.getHeading(AngleUnit.DEGREES));
                packet.put("Degree", odo.getPosition().getHeading(AngleUnit.DEGREES));
                packet.put("Pos", odo.getPosition());
                odo.getPosition().getX(DistanceUnit.INCH);

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}
