package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mecanum.traction.MecanumTractionConfig;

public class ArmManipulator extends ManipulatorBase {

    protected LinearOpMode linearOpMode;

    protected DcMotor arm_angle;
    private DcMotor.Direction direction;

    protected static double mtr_accel_min = ArmAngleManipulatorConfig.mtr_accel_min;
    protected static double mtr_decel_min = ArmAngleManipulatorConfig.mtr_decel_min;
    protected static double mtr_accel_tics = ArmAngleManipulatorConfig.mtr_accel_tics;
    protected static double mtr_decel_tics = ArmAngleManipulatorConfig.mtr_decel_tics;
    protected static double tollerance = ArmAngleManipulatorConfig.tollerance;
    protected static double encoder_resolution = 537.7f;

    public ArmManipulator(DcMotor.Direction direction) {
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
