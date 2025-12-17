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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "AutoDrivingBLUE-Long", group = "imReady")
public class AutoDrivingBLUE_Long extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 초기 위치 설정 (예: (0, 0), 0도)
        Pose2d beginPose = new Pose2d(64.45, 12, Math.PI);
        MecanumBase drive = new MecanumBase(hardwareMap, beginPose);
        ShooterBase shooter = new ShooterBase(hardwareMap);
        CameraBase camera = CameraBase.getInstance(hardwareMap);
        int id = 20;


        // 슈팅 액션 정의
        Action shootAction = new Action() {
            private boolean initialized = false;
            private ElapsedTime time;
            private double startTime;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    shooter.setServoPosition(ShooterBase.SERVO_MIN);      // 혹시모르니 생성자 단계에서 서보 초기 위치로 초기화 안되면 주석해제

                    time = new ElapsedTime();
                    startTime = time.milliseconds();
                    initialized = true;
                }

                double elapsed = time.milliseconds() - startTime;
                AprilTagDetection detection = null;

                sleep(2000);

                for (int i=0; i<50; i++) {
                    camera.update();
                    drive.currentHeading = drive.getRoadRunnerDrive().localizer.getPose().heading.log();
                    if (camera.isTagVisible(id)) {
                        detection = camera.getDetectionById(id);
                        drive.targetHeading = drive.currentHeading - camera.getDetectionById(id).ftcPose.bearing + camera.BEARING_OFFSET;;
                    }
                    drive.move(0,0,0);
                }
                drive.targetHeading = drive.currentHeading;
                drive.move(0, 0, 0);
                if (detection != null) { shooter.activeShooterTps(detection); }
                else { shooter.setVelocity(980); }

                for (int i=0; i<4; i++) {
                    while (shooter.updateCycle()) {
                        shooter.pushBall();
                    }
                }
                return false;
            }
        };

        waitForStart();

        if (isStopRequested())
            return;

        Actions.runBlocking(
                new SequentialAction(
                        // 1. 이동: (61, 10, 200도) 위치로 이동 후 90도 회전
                        drive.getRoadRunnerDrive().actionBuilder(beginPose)
//                                .splineTo(new Vector2d(61, 10), Math.toRadians(200))
//                                .turnTo(200)
                                .turn(20)
                                .build(),

                        // 2. 발사
                        shootAction)

                        // 3. 로딩존: (64.45, 64.45)으로 복귀
//                        drive.getRoadRunnerDrive().actionBuilder(new Pose2d(61, 10, Math.toRadians(200)))
//                                .splineTo(new Vector2d(64.45, 64.45), Math.toRadians(180))
//                                .build())
        );
        camera.closeCamera();


    }
}
