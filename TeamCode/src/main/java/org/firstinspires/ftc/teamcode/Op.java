package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "Op")
public class Op extends OpMode {
    DcMotorEx m1, m2, m3, m4;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    public static double POWER = 0.5;


    @Override
    public void init() {
        // 모터 초기화
        m1 = hardwareMap.get(DcMotorEx.class, "rightBack");
        m2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        m3 = hardwareMap.get(DcMotorEx.class, "leftBack");
        m4 = hardwareMap.get(DcMotorEx.class, "leftFront");

        // 모터 방향 설정
        m1.setDirection(DcMotorEx.Direction.FORWARD);
        m2.setDirection(DcMotorEx.Direction.FORWARD);
        m3.setDirection(DcMotorEx.Direction.REVERSE);
        m4.setDirection(DcMotorEx.Direction.REVERSE);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    @Override
    public void loop() {
        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float rx = gamepad1.right_stick_x;

        float denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        m1.setPower(POWER * ((y + x + rx) / denominator));
        m2.setPower(POWER * ((y - x - rx) / denominator));
        m3.setPower(POWER * ((y - x + rx) / denominator));
        m4.setPower(POWER * ((y + x + rx) / denominator));
    }
}
