package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmManipulator extends ManipulatorBase {

    protected LinearOpMode linearOpMode;

    protected DcMotor arm_angle;
    protected DcMotor arm_length;
    protected Servo hand;

    protected int arm_angle_tics = 13420;


    protected void reset_arm_encoders() {
        arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }









    @Override
    public void initialize(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        HardwareMap hardware_map = linearOpMode.hardwareMap;

        // find the motors
        arm_angle = hardware_map.get(DcMotor.class, "arm_angle");
        arm_length = hardware_map.get(DcMotor.class, "arm_length");
        // find the servo
        hand = hardware_map.get(Servo.class, "hand");

        // initialize the motors
        DcMotor.RunMode run_mode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior at_zero_power = DcMotor.ZeroPowerBehavior.BRAKE;
        lclMotorSetup(arm_angle, DcMotor.Direction.FORWARD, run_mode, at_zero_power);
        lclMotorSetup(arm_length, DcMotor.Direction.FORWARD, run_mode, at_zero_power);
        // initialize the servo
        lclServoSetup(hand, Servo.Direction.FORWARD);
    }

    @Override
    public void angle(double degrees, double max_speed) {
//        power_accel_decel(arm_angle.getCurrentPosition(), )
    }

    @Override
    public void length(double len, double max_speed) {

    }
}
