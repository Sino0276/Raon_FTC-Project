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
        camera.Update();

        // gamepad1 (Drive)
        drive.Update(gamepad1);

        // gamepad2 (Shooter)
        shooter.Update(gamepad2);

//        shooter();
        // display data
        showData();
    }

    @Override
    public void stop() {
        super.stop();
        camera.closeCamera();
    }

    //    private void shooter() {
//        if (gamepad2.xWasPressed()) {
//
//            shooter.isShooterSpin = !shooter.isShooterSpin;
//            if (shooter.isShooterSpin) {
//                shooter.setVelocity(shooter.LEFT_TPS, shooter.RIGHT_TPS);
//            } else {
//                shooter.setPower(0, 0);
//            }
//        }
//        // 슈터 속도 조절
//        else if (gamepad2.dpadUpWasPressed())   { shooter.addTPS(10);}
//        else if (gamepad2.dpadDownWasPressed()) { shooter.addTPS(-10);}
//        else if (gamepad2.dpadLeftWasPressed()) { shooter.addTPS(1);}
//        else if (gamepad2.dpadRightWasPressed()){ shooter.addTPS(-1);}
//
//        // 서보 회전
//        if (gamepad2.yWasPressed()) {
//
//            shooter.isServoSpin = !shooter.isServoSpin;
//            if (shooter.isServoSpin) {
//                shooter.servoSpin(shooter.SERVO_MAX);        // 여기
//            } else {
//                shooter.servoSpin(shooter.SERVO_MIN);     // 여기
//            }
//        }
//    }
    private void showData() {
        multipleTelemetry.addData("Shooter TargetTPS", ShooterBase.TPS);
        multipleTelemetry.addData("Shooter CurrnetLeftTPS", shooter.shooterLeft.getVelocity());
        multipleTelemetry.addData("Shooter CurrnetRightTPS", shooter.shooterRight.getVelocity());
        multipleTelemetry.addData("Shooter CurrnetAvgTPS",
                (shooter.shooterLeft.getVelocity() + shooter.shooterRight.getVelocity())/2);
        multipleTelemetry.addData("AprilTag", camera.isTagVisible(20));
        multipleTelemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}