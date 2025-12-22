package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Disabled
@TeleOp(name = "CamTest", group = "Test")
public class CamTest extends OpMode {
    private final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
    private final String WEBCAM_NAME = "Webcam 1";
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        FtcDashboard.getInstance().startCameraStream(camera, 0);
//
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                camera.startStreaming(640, 480);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                // 카메라 열기 실패 처리
//            }
//        });

        aprilTag = new AprilTagProcessor.Builder()
//                .setLensIntrinsics()  // 캠 보정
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME));
            packet.put("Camera", "Initialized");
        } else { packet.put("Camera", "Not Initialized"); }

        // 카메라 해상도를 선택합니다. 모든 카메라가 모든 해상도를 지원하는 것은 아닙니다.
        //builder.setCameraResolution(new Size(640, 480));

        // RC 미리보기(LiveView)를 활성화합니다.  카메라 모니터링을 생략하려면 "false"로 설정하세요.
        builder.enableLiveView(true);

        // 스트림 형식을 설정합니다. MJPEG는 기본 YUY2보다 적은 대역폭을 사용합니다.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // 활성화된 프로세서가 없는 경우 LiveView를 중지할지 여부를 선택합니다.
        // "true"로 설정하면 활성화된 프로세서가 없으면 모니터에 주황색 화면이 표시됩니다.
        // "false"로 설정하면 모니터에 주석 없이 카메라 보기가 표시됩니다.
        //builder.setAutoStopLiveView(false);

        // 프로세서를 설정하고 활성화합니다.
        builder.addProcessor(aprilTag);

        // 위 설정을 사용하여 Vision Portal을 구축합니다.
        visionPortal = builder.build();

        // 언제든지 aprilTag 프로세서를 비활성화하거나 다시 활성화합니다.
        //visionPortal.setProcessorEnabled(aprilTag, true);

//        visionPortal.resumeStreaming();

        dashboard.startCameraStream(visionPortal, 10);

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void loop() {
        telemetryAprilTag();
//        dashboard.sendTelemetryPacket(packet);
    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        packet.put("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                packet.put("ID", detection.id);
                packet.put("Name", detection.metadata.name);
                packet.put("X Y Z (inch)", String.format("%6.1f %6.1f %6.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                packet.put("Pitch Roll Yaw (deg)", String.format("%6.1f %6.1f %6.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                packet.put("Range Bearing Elevation (inch, deg, deg)", String.format("%6.1f %6.1f %6.1f", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                packet.put("ID", detection.id);
                packet.put("Name", "Unknown");
                packet.put("Center (pixels)", String.format("%6.0f %6.0f", detection.center.x, detection.center.y));
            }
        }   // end for() loop
        dashboard.sendTelemetryPacket(packet);
    }
}
