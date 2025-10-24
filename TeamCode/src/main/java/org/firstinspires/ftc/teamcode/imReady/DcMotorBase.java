package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public abstract class DcMotorBase {

    public void motorSetup(DcMotorEx motor,
                           DcMotorEx.Direction direction,
                           DcMotorEx.RunMode runMode,
                           DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

//    public abstract void setVelocity();
}
