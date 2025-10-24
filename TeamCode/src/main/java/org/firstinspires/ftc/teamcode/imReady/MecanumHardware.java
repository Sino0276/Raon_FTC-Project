package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public abstract class MecanumHardware extends DcMotorBase {
    protected DcMotorEx mtr_rr, mtr_rf, mtr_lr, mtr_lf;
    protected List<DcMotorEx> motors;

    public MecanumHardware(HardwareMap hardwareMap) {
        mtr_rr = hardwareMap.get(DcMotorEx.class, "mtr_rr");
        mtr_rf = hardwareMap.get(DcMotorEx.class, "mtr_rf");
        mtr_lr = hardwareMap.get(DcMotorEx.class, "mtr_lr");
        mtr_lf = hardwareMap.get(DcMotorEx.class, "mtr_lf");

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        motorSetup(mtr_rr, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_rf, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_lr, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(mtr_lf, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);

        motors = Arrays.asList(mtr_rr, mtr_rf, mtr_lr, mtr_lf);
    }

//    public void motorSetup(DcMotorEx motor,
//                              DcMotorEx.Direction direction,
//                              DcMotorEx.RunMode runMode,
//                              DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
//        motor.setDirection(direction);
//        motor.setMode(runMode);
//        motor.setZeroPowerBehavior(zeroPowerBehavior);
//    }

}
