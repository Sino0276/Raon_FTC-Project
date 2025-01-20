package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "ParkingLeft", group = "test")
public class _ParkingLeft extends LinearOpMode {


    private ITraction traction = new MecanumPidTraction();

    @Override
    public void runOpMode() /*throws  InterruptedException*/ {
        // initialize the traction base.
        traction.initialize(this);

        this.waitForStart();
        traction.postStartInitialize();

        traction.move(10, 0, 1);
    }
}
