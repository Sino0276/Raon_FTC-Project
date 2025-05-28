package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface IMecanum {

    /**
     * Initialize the traction implementation, which normally means find the traction motors in the
     * <tt>linearOpMode</tt> and initialize them for use. Setup IMUs, etc.
     *
     * @param linearOpMode (LinearOpMode, readonly) The liner operation mode this traction is being used in.
     */
    void initialize(LinearOpMode linearOpMode);
}
