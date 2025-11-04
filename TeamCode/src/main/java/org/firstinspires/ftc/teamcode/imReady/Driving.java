package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private MecanumBase drive;
    private ShooterBase shooter;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        drive = new MecanumBase(hardwareMap);
        shooter = new ShooterBase(hardwareMap);
//        driveTrain = new SampleMecanumDrive(hardwareMap);

        dashboard.sendTelemetryPacket(packet);

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
        drive.updatePinpoint();
        drive.getCurrentYaw();

        if (gamepad1.dpad_up) {
            drive.targetYaw = drive.postYaw;
//            drive.rotate();
        } else if (gamepad1.dpad_left) {
            drive.targetYaw = drive.postYaw + 90;
//            drive.rotate();
        } else if (gamepad1.dpad_down) {
            drive.targetYaw = drive.postYaw + 180;
//            drive.rotate();
        } else if (gamepad1.dpad_right) {
            drive.targetYaw = drive.postYaw + 270;
//            drive.rotate();
        } else {

            double forward = gamepad1.left_stick_y;
            double sideways = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            drive.setSpeed(forward, sideways, rotate);
        }
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
        packet.put("Encoder_X", drive.pinpoint.getEncoderX());
        packet.put("Encoder_Y", drive.pinpoint.getEncoderY());
        packet.put("Pos_X", drive.pinpoint.getPosX(DistanceUnit.INCH));
        packet.put("Pos_Y", drive.pinpoint.getPosY(DistanceUnit.INCH));
//                packet.put("Velocity_X", odo.getVelX(DistanceUnit.INCH));
//                packet.put("Velocity_Y", odo.getVelY(DistanceUnit.INCH));
        packet.put("getHeading", drive.pinpoint.getHeading(AngleUnit.DEGREES));
        packet.put("pos.getHeading", drive.pinpoint.getPosition().getHeading(AngleUnit.DEGREES));
        packet.put("Pos", drive.pinpoint.getPosition());

        packet.put("targetYaw", drive.targetYaw);
        packet.put("currentYaw", drive.currentYaw);
        packet.put("postYaw", drive.postYaw);

        telemetry.addData("Shooter Left TPS", shooter.LEFT_TPS);
        telemetry.addData("Shooter RIGHT TPS", shooter.RIGHT_TPS);
        telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }
}