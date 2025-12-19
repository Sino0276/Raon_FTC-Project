package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@Config
@TeleOp(name = "mtrTest", group = "Test")
public class MoterTest extends OpMode {
    public static PIDFCoefficients DRIVE_PID = new PIDFCoefficients(5, 2, 1, 11);
    /// max TPS == 2800
    public static int TPS = 2800;
    /// targetPosition 허용 오차
    public static int TOLERANCE = 1;
    /// 1 inch = 45 tick
    public static int TARGET_POS = 45;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    DcMotorEx mtr;

    @Override
    public void init() {
        mtr = hardwareMap.get(DcMotorEx.class, "leftFront");
        mtrLoad();
    }

    @Override
    public void loop() {
        mtrLoad();

        if (gamepad1.a) {
            move();
        }

        showData();
    }

    private void mtrLoad() {
        mtr.setVelocityPIDFCoefficients(DRIVE_PID.p, DRIVE_PID.i, DRIVE_PID.d, DRIVE_PID.f);
        mtr.setTargetPositionTolerance(TOLERANCE);  // 허용오차 설정
    }

    /**
     * <p> 537.7 tick = 360 degree
     * <p> WheelRadius = 1.9
     * <p> Tick Per Inch = tick / (2 * WheelRadius * PI)
     * <p> = 537.7 / (2 * 1.9 * 3.14)
     * <p> = 45.0....
     */
    private void move() {
        mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mtr.setTargetPosition(TARGET_POS);

        mtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Velocity의 PIDF계수 사용X -> TargetPos의 P계수만 사용. 즉 TargetPos랑 Velocity를 같이 사용하면, Velocity는 최대속도 제한용 인거임
        // 근데 짜피 PinPoint사용하면 바퀴 굴릴땐 targetPosition못씀. 로봇팔에선 쓸 수 있겠네
        mtr.setVelocity(TPS);
    }

    private void showData() {
        packet.put("CurrentPos", mtr.getCurrentPosition());
        packet.put("TargetPos", mtr.getTargetPosition());
        packet.put("TPS", TPS);
        packet.put("Velocity", mtr.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }
}
