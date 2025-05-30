package org.firstinspires.ftc.teamcode.mecanum;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mecanum.manipulator.ArmAngleManipulator;
import org.firstinspires.ftc.teamcode.mecanum.traction.ITraction;
import org.firstinspires.ftc.teamcode.mecanum.traction.MecanumPidTraction;

@TeleOp(name = "Driving")
public class Driving extends LinearOpMode {
//    private int drive_mode = 2;
//    private int last_drive_mode = drive_mode;
//    private String[] drive_mode_name = {"tank", "airplane right", "airplane left", "auto right", "auto left"};

    private MecanumPidTraction traction = new MecanumPidTraction();      // Mecanum, with PID heading correction
    private ArmAngleManipulator angle = new ArmAngleManipulator(DcMotorSimple.Direction.REVERSE, 1);

    private double bumper_speed = 0.6;
    double calibration_distance = 24.0;
    // gamepad1 state
    private double left1_x = 0.0;
    private double left1_y = 0.0;
    private double right1_x = 0.0;
    private double right1_y = 0.0;
    // gamepad2 state
    private double left2_x = 0.0;
    private double left2_y = 0.0;
    private double right2_x = 0.0;
    private double right2_y = 0.0;

    @Override
    public void runOpMode() {
        // initialize the mecanum traction base.
        traction.initialize(this);
        angle.initialize(this);
        this.waitForStart();
        traction.postStartInitialize();

        while (this.opModeIsActive()) {
            if (gamepad1.x) {
                // Test for finished
                break;
            } else if (gamepad1.y) {
                // move in a 30 square rotated 30 degrees
                traction.move(12.0,30.0,1.0);
                traction.move(-12.0,-60.0,1.0);
                traction.move(-12.0,30.0,1.0);
                traction.move(12.0,-60.0,1.0);
            } else if (gamepad1.dpad_up) {
                // Move forward the calibration distance
                traction.move(calibration_distance,0.0,1.0);
            } else if (gamepad1.dpad_down) {
                // Move backwards the calibration distance
//                traction.move(calibration_distance,180.0,1.0);
                traction.move(-calibration_distance,0.0,1.0);
            } else if (gamepad1.dpad_right) {
                if (gamepad1.right_bumper) {
                    // Rotate 90 clockwise
                    traction.rotate(90.0, 1.0);
                } else {
                    // Move right the calibration distance
                    traction.move(calibration_distance,90.0,1.0);
                }
            } else if (gamepad1.dpad_left) {
                if (gamepad1.right_bumper) {
                    // Rotate 90 counter-clockwise
                    traction.rotate(-90.0, 1.0);
                } else {
                    // Move left the calibration distance
//                    traction.move(calibration_distance,-90.0,1.0);
                    traction.move(-calibration_distance, 90.0, 1.0);
                }
            } else if (gamepad2.dpad_up) {
                angle.move(90, 0.5);
            } else if (gamepad2.dpad_down) {
                angle.move(-90, 0.5);
            } else {
                conditionSticks();
                traction.setSpeeds(left1_y, left1_x, right1_x);
                angle.setSpeeds(left2_y);

            }
            telemetry.update();
        }
    }

    private void conditionSticks() {
        left1_x = gamepad1.left_stick_x;
        left1_y = -gamepad1.left_stick_y;
        right1_x = gamepad1.right_stick_x;
        right1_y = -gamepad1.right_stick_y;

        left2_x = gamepad2.left_stick_x;
        left2_y = gamepad2.left_stick_y;
        right2_x = gamepad2.right_stick_x;
        right2_y = gamepad2.right_stick_y;
        if (gamepad1.left_bumper) {
            left1_x *= bumper_speed;
            left1_y *= bumper_speed;
            right1_x *= bumper_speed;
            right1_y *= bumper_speed;
        }
    }
}
