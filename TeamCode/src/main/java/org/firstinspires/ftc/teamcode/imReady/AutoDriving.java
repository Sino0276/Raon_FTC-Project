package org.firstinspires.ftc.teamcode.imReady;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@Disabled
@Autonomous(name = "AutoDriving", group = "imReady")
public class AutoDriving extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 초기 위치 설정 (예: (0, 0), 0도)
        Pose2d beginPose = new Pose2d(64.45, 12, Math.PI);
        MecanumBase drive = new MecanumBase(hardwareMap, beginPose);
        ShooterBase shooter = new ShooterBase(hardwareMap);
        CameraBase camera = CameraBase.getInstance(hardwareMap);

        // 슈팅 액션 정의
        Action shootAction = new Action() {
            private boolean initialized = false;
            private ElapsedTime time;
            private double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
//                    shooter.setServoPosition(ShooterBase.SERVO_MIN);      // 혹시모르니 생성자 단계에서 서보 초기 위치로 초기화 안되면 주석해제

                    time = new ElapsedTime();
                    startTime = time.milliseconds();
                    initialized = true;
                }

                double elapsed = time.milliseconds() - startTime;

                if (elapsed < 1000) {
                    // 1초 동안 모터 가속 대기
                    return true;
                } else if (elapsed < 2000) {
                    // 1초 후 서보 작동 (발사)
                    shooter.setServoPosition(ShooterBase.SERVO_MAX);
                    return true;
                } else if (elapsed < 2500) {
                    // 0.5초 후 서보 복귀
                    shooter.setServoPosition(ShooterBase.SERVO_MIN);
                    return true;
                } else {
                    // 종료: 모터 정지
                    shooter.setPower(0);
                    return false;
                }
            }
        };

        waitForStart();

        if (isStopRequested())
            return;

        Actions.runBlocking(
                new SequentialAction(
                        // 1. 이동: (10, 10) 위치로 이동 후 90도 회전
                        drive.getRoadRunnerDrive().actionBuilder(beginPose)
                                .splineTo(new Vector2d(10, 10), Math.toRadians(90))
                                .build(),

                        // 2. 발사
                        shootAction,

                        // 3. 복귀: (0, 0)으로 복귀
                        drive.getRoadRunnerDrive().actionBuilder(new Pose2d(10, 10, Math.toRadians(90)))
                                .splineTo(new Vector2d(0, 0), Math.toRadians(0))
                                .build()));
    }
}
