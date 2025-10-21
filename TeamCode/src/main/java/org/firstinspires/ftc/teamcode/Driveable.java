package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
public abstract class Driveable extends OpMode {

    public static double V_forward_V_side_bias = 0.001;
    public static double V_turn_V_side_bias = 0.001;
    public static int TPS = 2800;
    public static double SCALE = 0.7;

    private DcMotorEx mtr_rr, mtr_rf, mtr_lr, mtr_lf;
    private List<DcMotorEx> motors;

    protected void mtrInit() {
        mtr_rr = hardwareMap.get(DcMotorEx.class, "mtr_rr");
        mtr_rf = hardwareMap.get(DcMotorEx.class, "mtr_rf");
        mtr_lr = hardwareMap.get(DcMotorEx.class, "mtr_lr");
        mtr_lf = hardwareMap.get(DcMotorEx.class, "mtr_lf");

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        motorSetup(mtr_rr, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(mtr_rf, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(mtr_lr, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_lf, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);

        motors = Arrays.asList(mtr_rr, mtr_rf, mtr_lr, mtr_lf);
    }

    protected void motorSetup(DcMotorEx motor,
                              DcMotorEx.Direction direction,
                              DcMotorEx.RunMode runMode,
                              DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setVelocity(double power_rr, double power_rf, double power_lr, double power_lf) {
        mtr_rr.setVelocity(TPS * power_rr);
        mtr_rf.setVelocity(TPS * power_rf);
        mtr_lr.setVelocity(TPS * power_lr);
        mtr_lf.setVelocity(TPS * power_lf);
    }


    public void setSpeeds(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        SCALE = 0.7;
        double max = Math.abs(forward) +
                    Math.abs(sideways * (Math.abs(V_forward_V_side_bias) + Math.abs(V_turn_V_side_bias) + 1.0)) +
                Math.abs(rotate);
        if (max > 1.0) {
            SCALE = 1.0 / max;
        }
        // Compute the power to each of the motors
        double power_rf = SCALE *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias - 1.0) -
                        rotate);
        double power_rr = SCALE *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias + 1.0) -
                        rotate);
        double power_lf = SCALE *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias + 1.0) +
                        rotate);
        double power_lr = SCALE *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias - 1.0) +
                        rotate);
        // set the powers to each of the motors
        setVelocity(power_rf, power_rr, power_lf, power_lr);
    }
}
