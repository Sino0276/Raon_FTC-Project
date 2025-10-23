package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public abstract class ShooterHardware {
    protected DcMotorEx shooter1, shooter2;
    protected List<DcMotorEx> shooters;

    public ShooterHardware(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        motorSetup(shooter1, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(shooter2, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);

        shooters = Arrays.asList(shooter1, shooter2);
    }

    public void motorSetup(DcMotorEx motor,
                           DcMotorEx.Direction direction,
                           DcMotorEx.RunMode runMode,
                           DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
