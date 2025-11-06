package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class CameraBase extends CameraHardware {
    private final int APRILTAG_FIRST_ID = 20;

    public CameraBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void enableCameraStreaming(int maxFps) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, maxFps);
    }

    public void detactAprilTags() {
        currentDetections = aprilTag.getDetections();
//        currentDetections.sort();
    }
}
