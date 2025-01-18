package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

@Autonomous(name = "ParkingRight", group = "")
public class _ParkingRight extends LinearOpMode {

    private ITraction traction = new MecanumPidTraction();

    @Override
    public void runOpMode()  throws java.lang.InterruptedException {
        // initialize the traction base.
        traction.initialize(this);

        this.waitForStart();
        traction.postStartInitialize();

        traction.move(40, -90, 0.7);
    }
}
