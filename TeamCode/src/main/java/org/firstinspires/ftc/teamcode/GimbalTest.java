package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.imReady.CameraBase;
import org.firstinspires.ftc.teamcode.imReady.MecanumBase;

import java.util.ArrayList;

@Config
@TeleOp(name = "GimbalTest")
public class GimbalTest extends OpMode {
    // 설정 변수 (대시보드에서 수정 가능)
    public static int TAG_ID = 24; // 테스트할 태그 ID
    public static double TEST_DURATION_SEC = 5.0; // 테스트 진행 시간 (초)
    public static boolean startTest = false; // 테스트 시작 트리거

    private CameraBase camera;
    private MecanumBase drive;
    private FtcDashboard dashboard;
    private MultipleTelemetry multipleTelemetry;

    // 테스트 상태 관리
    private enum TestState {
        IDLE,       // 대기 중
        RUNNING,    // 테스트 진행 중
        FINISHED    // 결과 출력
    }
    private TestState currentState = TestState.IDLE;

    private ElapsedTime timer = new ElapsedTime();

    // 데이터 집계 변수
    private long totalLoops = 0;
    private long detectedLoops = 0;
    private double finalAccuracy = 0.0;

    @Override
    public void init() {
        camera = CameraBase.getInstance(hardwareMap);
        // FPS는 짐벌 테스트 시 중요하므로 가능한 높게 설정
        camera.enableCameraStreaming(60);
        drive = new MecanumBase(hardwareMap, new Pose2d(0, 0 , 0));

        dashboard = FtcDashboard.getInstance();
        multipleTelemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        multipleTelemetry.addData("State", "Initialized");
        multipleTelemetry.addData("Instruction", "Dashboard에서 'startTest'를 true로 체크하세요.");
        multipleTelemetry.update();
    }

    @Override
    public void loop() {
        // 매 루프마다 카메라 데이터 갱신
        camera.update();

        switch (currentState) {
            case IDLE:
                // 대기 상태 로직
                multipleTelemetry.addData("Status", "Ready to start");
                if (startTest) {
                    startTest = false; // 트리거 리셋
                    startNewTest();
                }
                break;

            case RUNNING:
                // 테스트 진행 로직
                drive.update(gamepad1);
                if (timer.seconds() < TEST_DURATION_SEC) {
                    // 데이터 수집
                    totalLoops++;
                    if (camera.isTagVisible(TAG_ID)) {
                        detectedLoops++;
                    }

                    multipleTelemetry.addData("Status", "Testing...");
                    multipleTelemetry.addData("Time Remaining", "%.1f sec", TEST_DURATION_SEC - timer.seconds());
                } else {
                    // 시간 종료 시 결과 계산
                    finishTest();
                }
                break;

            case FINISHED:
                drive.move(0, 0, 0);
                // 결과 표시 로직
                multipleTelemetry.addData("Status", "Finished");
                multipleTelemetry.addData("--- 결과 리포트 ---", "");
                multipleTelemetry.addData("총 샘플 수(Loops)", totalLoops);
                multipleTelemetry.addData("성공 횟수", detectedLoops);
                multipleTelemetry.addData("인식률(Accuracy)", "%.2f %%", finalAccuracy * 100);

                // 다시 시작하고 싶으면
                if (startTest) {
                    startTest = false;
                    startNewTest();
                }
                break;
        }

        // 현재 태그 상태 실시간 모니터링 (디버깅용)
        multipleTelemetry.addData("Current Tag Visible", camera.isTagVisible(TAG_ID));
        multipleTelemetry.update();
    }

    private void startNewTest() {
        totalLoops = 0;
        detectedLoops = 0;
        finalAccuracy = 0.0;
        timer.reset();
        currentState = TestState.RUNNING;
    }

    private void finishTest() {
        if (totalLoops > 0) {
            // 정수 나눗셈 문제 해결: (double) 캐스팅
            finalAccuracy = (double) detectedLoops / totalLoops;
        } else {
            finalAccuracy = 0.0;
        }
        currentState = TestState.FINISHED;
    }

    @Override
    public void stop() {
        super.stop();
        camera.closeCamera();
    }
}
