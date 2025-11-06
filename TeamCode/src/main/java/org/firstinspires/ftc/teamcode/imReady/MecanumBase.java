package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class MecanumBase extends MecanumHardware {
    public static double V_forward_V_side_bias = 0.001;
    public static double V_turn_V_side_bias = 0.001;
    public static int TPS = 2000;
    public static double kp = 0.02;


    public double currentYaw, targetYaw, postYaw;

    public MecanumBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        updatePinpoint();
        currentYaw = targetYaw = postYaw = pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public void setVelocity(double power_rr, double power_rf, double power_lr, double power_lf) {
        mtr_rr.setVelocity(TPS * power_rr);
        mtr_rf.setVelocity(TPS * power_rf);
        mtr_lr.setVelocity(TPS * power_lr);
        mtr_lf.setVelocity(TPS * power_lf);
    }

    public void setSpeed(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        double scale = 1;
        double max = Math.abs(forward) +
                Math.abs(sideways * (Math.abs(V_forward_V_side_bias) + Math.abs(V_turn_V_side_bias) + 1.0)) +
                Math.abs(rotate);
        if (max > 1.0) {
            scale = 1.0 / max;
        }

        if (rotate == 0) {/// ///////////////////////////////////
            double error = targetYaw - currentYaw;
            if (error > 180) {
                error -= 360;
            } else if (error <= -180) {
                error += 360;
            }
            rotate = kp * error;
        } else {
            currentYaw = targetYaw = getCurrentYaw();
        }

        // Compute the power to each of the motors
        double power_rf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias - 1.0) -
                        rotate);
        double power_rr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias - V_turn_V_side_bias + 1.0) -
                        rotate);
        double power_lf = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias + 1.0) +
                        rotate);
        double power_lr = scale *
                (forward +
                        sideways * (V_forward_V_side_bias + V_turn_V_side_bias - 1.0) +
                        rotate);
        // set the powers to each of the motors
        setVelocity(power_rf, power_rr, power_lf, power_lr);
    }

//    public void move(double inches, double degrees, double max_speed) {
//
//    }

    public void rotate() {
        double error = targetYaw - currentYaw;
        if (error > 180) {
            error -= 360;
        } else if (error <= -180) {
            error += 360;
        }
        double rotate = kp * error;

        double power_rf = -rotate;
        double power_rr = -rotate;
        double power_lf = rotate;
        double power_lr = rotate;

        setVelocity(power_rf, power_rr, power_lf, power_lr);
    }

    public void setTPS(int TPS) {
        if (TPS > 2800) { MecanumBase.TPS = 2800; }
        else if (TPS < 0) { MecanumBase.TPS = 0; }
        else { MecanumBase.TPS = TPS; }
    }
    public void addTPS(int amount) { setTPS(TPS + amount); }

    public double getCurrentYaw() {
        currentYaw = pinpoint.getHeading(AngleUnit.DEGREES);
        return currentYaw;
    }

    public void updatePinpoint() {
        pinpoint.update();
    }
}
