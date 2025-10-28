package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class MecanumHardware {
    protected DcMotorEx mtr_rr, mtr_rf, mtr_lr, mtr_lf;
    protected GoBildaPinpointDriver pinpoint;

    public MecanumHardware(HardwareMap hardwareMap) {
        mtr_rr = hardwareMap.get(DcMotorEx.class, "rightBack");
        mtr_rf = hardwareMap.get(DcMotorEx.class, "rightFront");
        mtr_lr = hardwareMap.get(DcMotorEx.class, "leftBack");
        mtr_lf = hardwareMap.get(DcMotorEx.class, "leftFront");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        initMotor();
        initPinpoint();
    }

    private void initPinpoint() {
        pinpoint.initialize();
        pinpoint.setOffsets(3, -72, DistanceUnit.MM);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        pinpoint.recalibrateIMU();
        pinpoint.update();
    }
    private void initMotor() {
        DcMotor.RunMode runMode = DcMotor.RunMode.RUN_USING_ENCODER;
        DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

        motorSetup(mtr_rr, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_rf, DcMotorEx.Direction.REVERSE, runMode, zeroPowerBehavior);
        motorSetup(mtr_lr, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
        motorSetup(mtr_lf, DcMotorEx.Direction.FORWARD, runMode, zeroPowerBehavior);
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
