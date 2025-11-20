package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private CameraBase camera;
    private MecanumBase drive;
    private ShooterBase shooter;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private MultipleTelemetry multipleTelemetry;

    @Override
    public void init() {
        camera = CameraBase.getInstance(hardwareMap);
        drive = new MecanumBase(hardwareMap, new Pose2d(0, 0, 0));
        shooter = new ShooterBase(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        camera.enableCameraStreaming(30);
        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void loop() {
        // camera
        camera.update();

        // gamepad1 (Drive)
        drive.update(gamepad1);

        // gamepad2 (Shooter)
        shooter.update(gamepad2);

        // display data
        showData();
    }

    @Override
    public void stop() {
        super.stop();
        camera.closeCamera();
    }

    private void showData() {
        multipleTelemetry.addData("Shooter TargetTPS", ShooterBase.TPS);
        multipleTelemetry.addData("Shooter CurrnetLeftTPS", shooter.shooterLeft.getVelocity());
        multipleTelemetry.addData("Shooter CurrnetRightTPS", shooter.shooterRight.getVelocity());
        multipleTelemetry.addData("Shooter CurrnetAvgTPS",
                (shooter.shooterLeft.getVelocity() + shooter.shooterRight.getVelocity()) / 2);
        multipleTelemetry.addData("AprilTag", camera.isTagVisible(20));
        multipleTelemetry.addData("Shooter Efficency", ShooterBase.SHOOTER_EFFICENCY);
        multipleTelemetry.addData("Distance",
                camera.isTagVisible(20) ? camera.getDetectionById(20).ftcPose.y : "No Tag");
        multipleTelemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}