package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "Position PIDF")
@Disabled
public class PositionPIDF extends OpMode {

    public static double KP = 0.0035;       // 비례 상수
    public static double KI = 0.0001;       // 적분 상수
    public static double KD = 0.0000001;    // 미분 상수
    public static double KF = 0.14;         // 피드포워드 상수
    public static int TARGET_POS = 0;       // 목표 위치
    public static boolean MOTOR_ON = false; // 모터 on / off


    private int lastTarget = TARGET_POS;
    private double currentPos;      // 현재 위치
    private double lastError = 0;   // 이전 오차 값
    private double integral = 0;    // 적분 값
    private ElapsedTime timer;      // 타이머 (dt 구할 때 사용)
    private double lastTime = 0;
    private double TICKS_PER_REVOLUTION = 537.7; // 1회전당 틱 수
    private double GEAR_RATIO = 5; // 기어비
    private double TICKS_PER_RADIAN = (TICKS_PER_REVOLUTION * GEAR_RATIO) / (2 * Math.PI);  // 1라디안당 틱 수

    private DcMotorEx motor;
    private FtcDashboard dashboard;
    private MultipleTelemetry multipleTelemetry;



    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "leftFront");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);    // 모터 정지 및 인코더 초기화
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);       // 인코더 사용X (= 내부 PIDF 제어기 사용X)

        timer = new ElapsedTime();

        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }

    @Override
    public void loop() {
        double power = 0;
        currentPos = motor.getCurrentPosition();

        if (MOTOR_ON) {


            if (TARGET_POS != lastTarget) {
                integral = 0; // 목표 변경 시 적분 리셋
                lastTarget = TARGET_POS;
            }

            power = calculatePIDF(TARGET_POS, currentPos);
            motor.setPower(power);
        }
        else {
            motor.setPower(0);
            timer.reset();
            lastTime = timer.seconds();
            lastError = 0;
            integral = 0; // 모터 정지 시 적분 리셋
        }


        // 디버그 정보 출력
        multipleTelemetry.addData("Target Pos", TARGET_POS);
        multipleTelemetry.addData("Current Pos", currentPos);
        multipleTelemetry.addData("Power", power);
        multipleTelemetry.addData("error", TARGET_POS - currentPos);
        multipleTelemetry.addData("i", integral);
    }

    /**
     * Position PIDF 제어 알고리즘
     *
     * @param targetPosition    목표 위치
     * @param currentPosition   현재 위치
     * @return                  최종 제어 출력
     */
    private double calculatePIDF(double targetPosition, double currentPosition) {
        // PIDF 제어 알고리즘 구현
        double error = targetPosition - currentPosition;
        double currentAngleInRadians = motor.getCurrentPosition() / TICKS_PER_RADIAN;
        multipleTelemetry.addData("rad", currentAngleInRadians);

        // 시간 변화량 계산
        // FTC 루프는 불규칙 (15ms~50ms)하므로 실제 시간 측정 필요
        // dt 없이 integral += error만 하면 루프 속도에 따라 적분값이 달라짐
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // error * dt = 오차 * 시간 = 오차의 면적 (정확한 적분)
        integral += error * dt;
        // 적분 제한 (Anti-windup)
//        integral = Math.max(-integralMax, Math.min(integralMax, integral));

        // 미분 계산
        // (error - lastError) / dt = 오차 변화율 (시간 독립적)
        double derivative = 0;
        if (dt > 0) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        double feedforward =  KF * Math.cos(currentAngleInRadians);
        // PIDF 출력 계산
        double output = KP * error + KI * integral + KD * derivative + feedforward;
        output = Range.clip(output, -1, 1);

        return output;
    }
}
