package org.firstinspires.ftc.teamcode.mecanum.manipulation;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmManipulation extends ManipulationBase {

    protected LinearOpMode linearOpMode;

    protected DcMotor arm_angle;
    private DcMotor.Direction direction;

    protected static double mtr_accel_min = ArmAngleManipulationConfig.mtr_accel_min;
    protected static double mtr_decel_min = ArmAngleManipulationConfig.mtr_decel_min;
    protected static double mtr_accel_tics = ArmAngleManipulationConfig.mtr_accel_tics;
    protected static double mtr_decel_tics = ArmAngleManipulationConfig.mtr_decel_tics;
    protected static double tollerance = ArmAngleManipulationConfig.tollerance;
    protected static double encoder_resolution = 537.7f;

    public ArmManipulation(DcMotor.Direction direction) {
        this.direction = direction;
    }

    @Override
    public void initialize(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        HardwareMap hardware_map = linearOpMode.hardwareMap;

        // find the motors
        arm_angle = hardware_map.get(DcMotor.class, "arm_angle");

        // initialize the motors
        DcMotor.RunMode run_mode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior at_zero_power = DcMotor.ZeroPowerBehavior.BRAKE;
        lclMotorSetup(arm_angle, direction, run_mode, at_zero_power);
    }

    @Override
    protected void reset_encoders() {
        arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setSpeeds(double forward) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        double scale = 1.0;

        if (forward > 1.0) {
            forward = 1;
        }
        arm_angle.setPower(forward);
    }
}
