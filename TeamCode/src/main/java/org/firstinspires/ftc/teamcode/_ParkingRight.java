package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.mecanum.traction.ITraction;
import org.firstinspires.ftc.teamcode.mecanum.traction.MecanumPidTraction;

@Autonomous(name = "ParkingRight")
public class _ParkingRight extends LinearOpMode {

    private ITraction traction = new MecanumPidTraction();

    @Override
    public void runOpMode()/*  throws java.lang.InterruptedException*/ {
        traction.initialize(this);

        this.waitForStart();
        telemetry.addData("m0", 1);
        telemetry.update();
        traction.postStartInitialize();
        telemetry.addData("m1", 1);
        telemetry.update();

        traction.move(24, 0, 0.7);
        telemetry.addData("m2", 1);
        telemetry.update();

        sleep(1000);



    }
}
