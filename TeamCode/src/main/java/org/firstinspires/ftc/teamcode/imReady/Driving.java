package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private MecanumBase drive;
    private ShooterBase shooter;
//    private CameraBase camera;

    private TelemetryPacket packet;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        drive = new MecanumBase(hardwareMap, new Pose2d(0, 0, 0));
        shooter = new ShooterBase(hardwareMap);
//        camera = new CameraBase(hardwareMap);

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void loop() {
        drive.Update(gamepad1);

        // gamepad2 (Shooter) =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        shooter();

        showData();
    }

    private void shooter() {
        if (gamepad2.xWasPressed()) {

            shooter.isShooterSpin = !shooter.isShooterSpin;
            if (shooter.isShooterSpin) {
                shooter.setVelocity(shooter.LEFT_TPS, shooter.RIGHT_TPS);
            } else {
                shooter.setPower(0, 0);
            }
        }
        // 슈터 속도 조절
        else if (gamepad2.dpadUpWasPressed())   { shooter.addTPS(10);}
        else if (gamepad2.dpadDownWasPressed()) { shooter.addTPS(-10);}
        else if (gamepad2.dpadLeftWasPressed()) { shooter.addTPS(1);}
        else if (gamepad2.dpadRightWasPressed()){ shooter.addTPS(-1);}

        // 서보 회전
        if (gamepad2.yWasPressed()) {

            shooter.isServoSpin = !shooter.isServoSpin;
            if (shooter.isServoSpin) {
                shooter.servoSpin(shooter.SERVO_MAX);        // 여기
            } else {
                shooter.servoSpin(shooter.SERVO_MIN);     // 여기
            }
        }
    }
    private void showData() {

        packet.put("rotate", gamepad1.right_stick_x);

        telemetry.addData("Shooter Left TPS", shooter.LEFT_TPS);
        telemetry.addData("Shooter RIGHT TPS", shooter.RIGHT_TPS);
        telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}