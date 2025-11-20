package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public abstract class CameraHardware {
    private final boolean USE_WEBCAM = true; // 카메라 사용 여부
    private final String WEBCAM_NAME = "Webcam 1"; // 웹캠 이름
    protected FtcDashboard dashboard;
    public AprilTagProcessor aprilTag; // AprilTag 프로세서
    public VisionPortal visionPortal; // 비전 포털

    public CameraHardware(HardwareMap hardwareMap) {
        dashboard = FtcDashboard.getInstance();
        // AprilTag 프로세서 생성
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(
                        new Position(DistanceUnit.MM, 180, 100, -170, 0),
                        new YawPitchRollAngles(AngleUnit.RADIANS, 0, 0, 2 * Math.PI, 0))
                // .setLensIntrinsics() // 캠 보정 (필요시 활성화)
                .build();

        // 비전 포털 생성
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
        } else {
            throw new IllegalStateException("Camera not initialized");
        }

        // 카메라 해상도를 설정
        // builder.setCameraResolution(new Size(640, 480));

        // 스트림 형식을 설정합니다. MJPEG는 기본 YUY2보다 적은 대역폭을 사용합니다.
        // builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // 라이브 뷰 활성화
        builder.enableLiveView(true);
        // AprilTag 프로세서 추가
        builder.addProcessor(aprilTag);
        // 비전 포털 빌드
        visionPortal = builder.build();
        // dashboard.startCameraStream(visionPortal, 30); // 30 FPS로 스트리밍 시작
    }
}
