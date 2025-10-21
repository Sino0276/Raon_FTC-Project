package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "DriveTest", group = "java")
public class DriveTest extends Driveable {

    private double left_x, left_y, right_x;

    @Override
    public void init() {
        mtrInit();
    }

    @Override
    public void loop() {
        getGamepadValue();
        setSpeeds(left_y, left_x, right_x);


    }

    private void getGamepadValue() {
        left_x = gamepad1.left_stick_x;
        left_y = gamepad1.left_stick_y;
        right_x = gamepad1.right_stick_x;
    }
}

