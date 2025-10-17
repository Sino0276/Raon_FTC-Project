package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.configs.ShooterConfig.TPS;
import static org.firstinspires.ftc.teamcode.configs.ShooterConfig.TURNING_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "mtrTest", group = "java")
public class driving extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();
    DcMotorEx mtr;

    @Override
    public void init() {
        mtr = hardwareMap.get(DcMotorEx.class, "mtr");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            /*
                537.7 tick = 360 degree
                WheelRadius = 1.9
                Tick Per Inch = tick / (2 * WheelRadius * PI)
                 = 537.7 / (2 * 1.9 * 3.14)
                 = 45

             */
            mtr.setTargetPosition(45);

            mtr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            mtr.setVelocity(TPS);
        }
        else if (gamepad1.b) {
            mtr.setVelocityPIDFCoefficients(TURNING_PID.p, TURNING_PID.i, TURNING_PID.d, TURNING_PID.f);
        }

        packet.put("CurrentPos", mtr.getCurrentPosition());
        packet.put("TargetPos", mtr.getTargetPosition());

        packet.put("TARGET", TPS);
        packet.put("leftVelocity", mtr.getVelocity());
        dashboard.sendTelemetryPacket(packet);
    }
}
