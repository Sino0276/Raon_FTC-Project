package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public abstract class ShooterHardware {
    protected DcMotorEx shooterLeft, shooterRight;
    protected Servo servo;

    public ShooterHardware(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);

        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;

        motorSetup(shooterLeft, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(shooterRight, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
    }

    private void motorSetup(DcMotorEx motor,
            DcMotorEx.Direction direction,
            DcMotorEx.RunMode runMode,
            DcMotorEx.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setDirection(direction);
        motor.setMode(runMode);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
}
