package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDF")
public class PIDF extends OpMode {

    public static double KP = 5; // 비례 상수
    public static double KI = 3; // 적분 상수
    public static double KD = 5; // 미분 상수
    public static double KF = 8; // 피드포워드 상수
    public static int TARGET_POS = 400;
    private double currentPos;

    private double previousError = 0; // 이전 오차 값 초기화
//    private double targetTPS = 1000; // 목표 TPS 값

    private DcMotorEx motor;

    private FtcDashboard dashboard;
    private TelemetryPacket packet;


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        currentPos = motor.getCurrentPosition();
    }

    @Override
    public void loop() {
        motor.setVelocityPIDFCoefficients(KP, KI, KD, KF); // PIDF 계수를 0으로 설정하여 수동 제어
        currentPos = motor.getCurrentPosition();
//        double tps = calculatePIDF(TARGET_POS, currentPos);
//        motor.setPower(tps/TARGET_POS);
        motor.setTargetPosition(TARGET_POS);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);

        // 디버그 정보 출력
        packet.put("Target TPS", TARGET_POS);
        packet.put("Current Pos", currentPos);
//        packet.put("Motor Tps", tps);
        packet.put("error", TARGET_POS - currentPos);
        dashboard.sendTelemetryPacket(packet);


    }

    private double calculatePIDF(double targetTPS, double currentTPS) {
        // PIDF 제어 알고리즘 구현


        double error = targetTPS - currentTPS;
        double integral = 0; // 적분 값 초기화
        double derivative = 0; // 미분 값 초기화
//        double previousError = 0; // 이전 오차 값 초기화

        integral += error; // 적분 계산
        derivative = error - previousError; // 미분 계산

        double output = (KP * error) + (KI * integral) + (KD * derivative) + (KF * targetTPS);

        previousError = error; // 이전 오차 값 업데이트

        return output;
    }
}
