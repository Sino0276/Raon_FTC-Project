package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.configs.ShooterConfig.TPS;
import static org.firstinspires.ftc.teamcode.configs.ShooterConfig.SHOOTER_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name = "ShooterTest", group = "java")
public class ShooterTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotorEx flyWheel1;
        DcMotorEx flyWheel2;
//        Servo servo;
        CRServo crServo;

        double leftStickY;

        waitForStart();
        if (opModeIsActive()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            flyWheel1 = hardwareMap.get(DcMotorEx.class, "r");
            flyWheel2 = hardwareMap.get(DcMotorEx.class, "l");
            flyWheel1.setDirection(DcMotorEx.Direction.REVERSE);
            flyWheel2.setDirection(DcMotorEx.Direction.FORWARD);
            flyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flyWheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flyWheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flyWheel1.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);
            flyWheel2.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);

//            servo = hardwareMap.get(Servo.class, "servo");
            crServo = hardwareMap.get(CRServo.class, "servo");


            while (opModeIsActive()) {

                leftStickY = gamepad1.left_stick_y;

                flyWheel1.setPower(leftStickY);
                flyWheel2.setPower(leftStickY);

                if (gamepad1.a) {
//                    servo.setPosition();
                    crServo.setPower(1);
                } else if (gamepad1.b) {
                    crServo.setPower(0);
                } else if (gamepad1.x) {
                    flyWheel1.setVelocity(TPS);
                    flyWheel2.setVelocity(TPS);
                } else if (gamepad1.y) {
                    dashboard.updateConfig();
                    flyWheel1.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);
                    flyWheel2.setVelocityPIDFCoefficients(SHOOTER_PID.p, SHOOTER_PID.i, SHOOTER_PID.d, SHOOTER_PID.f);
                }


                packet.put("TARGET", TPS);
                packet.put("leftVelocity", flyWheel1.getVelocity());
                packet.put("rightVelocity", flyWheel2.getVelocity());
                packet.put("leftEncoder", flyWheel1.getCurrentPosition());
                packet.put("rightEncoder", flyWheel2.getCurrentPosition());

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}
