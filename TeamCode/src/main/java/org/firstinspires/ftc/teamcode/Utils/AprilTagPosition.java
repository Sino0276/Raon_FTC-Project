package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.LUT;
import com.pedropathing.geometry.Pose;

public class AprilTagPosition {
    // Pedro Pathing 좌표계 기준으로 작성 (단위: mm)
    // Pedro Pathing 튜닝후 작성 할 것
    public static LUT<Integer, Pose> APRILTAG_POS = new LUT<Integer, Pose>()
    {{
        add(20, new Pose(0, 0, 0));
        add(24, new Pose(0, 0, 0));
    }};

}
