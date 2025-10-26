package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private MecanumBase drive;
    private ShooterBase shooter;
//    private SampleMecanumDrive driveTrain;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        drive = new MecanumBase(hardwareMap);
        shooter = new ShooterBase(hardwareMap);
//        driveTrain = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        drive();
        // gamepad2 (Shooter) =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        shooter();

        showData();
    }

    private void drive()
    {
        drive.getCurrentYaw();
        if (gamepad1.dpad_up) {
            drive.targetYaw = drive.postYaw;
            drive.rotate(0.7);
        } else if (gamepad1.dpad_left) {
            drive.targetYaw = drive.postYaw + 90;
            drive.rotate(0.7);
        } else if (gamepad1.dpad_down) {
            drive.targetYaw = drive.postYaw + 180;
            drive.rotate(0.7);
        } else if (gamepad1.dpad_right) {
            drive.targetYaw = drive.postYaw + 270;
            drive.rotate(0.7);
        } else {

            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            drive.setSpeed(forward, sideways, rotate);
        }
    }

    private void shooter() {
        if (gamepad2.aWasPressed()) {

            shooter.isShooterSpin = !shooter.isShooterSpin;
            if (shooter.isShooterSpin) {
                shooter.setVelocity(shooter.TPS);
            } else {
                shooter.setPower(0);
            }
        }
        // 슈터 속도 조절
        else if (gamepad2.dpadUpWasPressed()) { shooter.addTPS(100);}
        else if (gamepad2.dpadDownWasPressed()) { shooter.addTPS(-100);}

        // 서보 회전
        if (gamepad2.bWasPressed()) {

            shooter.isServoSpin = !shooter.isServoSpin;
            if (shooter.isServoSpin) {
                shooter.servoSpin(1);        // 여기
            } else {
                shooter.servoSpin(0);     // 여기
            }
        }
    }

    private void showData() {
        packet.put("TPS(Shooter)", shooter.TPS);
        packet.put("TPS(Drive)", drive.TPS);
        dashboard.sendTelemetryPacket(packet);
    }
}