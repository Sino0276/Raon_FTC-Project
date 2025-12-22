package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.LUT;
import com.pedropathing.geometry.Pose;

public class AprilTagPosition {
    public static LUT<Integer, Pose> APRILTAG_POS = new LUT<Integer, Pose>()
    {{
        add(20, new Pose(0, 0, 0));
        add(24, new Pose(0, 0, 0));
    }}
            ;




    public AprilTagPosition() {
    }
}
