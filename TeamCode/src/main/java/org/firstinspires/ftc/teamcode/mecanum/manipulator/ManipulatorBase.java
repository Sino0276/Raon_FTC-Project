package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ManipulatorBase implements IManipulator {

    protected void lclMotorSetup(DcMotor motor, DcMotor.Direction direction,
                                 DcMotor.RunMode run_mode, DcMotor.ZeroPowerBehavior zero_power_behavior) {
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(run_mode);
        motor.setZeroPowerBehavior(zero_power_behavior);
    }

    protected void lclServoSetup(Servo servo, Servo.Direction direction) {
        servo.setDirection(direction);
    }

    protected double power_accel_decel(double current, double target,
                                       double mtr_accel_min, double mtr_decel_min,
                                       double accel, double decel) {
        if (current <= 0.0) {
            // Not yet at the expected start. This could happen if there was some robot
            // motion (was hit or coasting) that confused the sensor/logic. In this
            // case, move at the minimum power until the caller knows what's happening.
            return mtr_accel_min;
        } else if (current >= target) {
            // Past the expected target. This could happen if there was some robot motion
            // (was hit or coasting) that confused the sensor/logic. In this case stop.
            return 0.0;
        }
        double mtr_tmp = 1.0;
        if (current < accel) {
            // in the acceleration zone
            mtr_tmp = mtr_accel_min + (1.0 - mtr_accel_min) * (current / accel);
        }
        if (current > target - decel) {
            // in the deceleration zone
            double mtr_tmp_2 = mtr_decel_min +
                    (1.0 - mtr_decel_min) * ((target - current) / decel);
            if (mtr_tmp_2 < mtr_tmp) {
                // Could also be in the acceleration zone - in this case the deceleration
                // value is less than the acceleration or the 1.0 default.
                mtr_tmp = mtr_tmp_2;
            }
        }
        return mtr_tmp;
    }
}
