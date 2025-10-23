package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class MecanumBase extends MecanumHardware {
    public static double V_forward_V_side_bias = 0.001;
    public static double V_turn_V_side_bias = 0.001;
    public static int TPS = 2800;
    public static double SCALE = 0.7;

    public enum DriveMode {
        TWOFINGER,
        FOURFINGER
    }
    private DriveMode driveMode = DriveMode.FOURFINGER;

    public MecanumBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void setVelocity(double power_rr, double power_rf, double power_lr, double power_lf) {
        mtr_rr.setVelocity(TPS * power_rr);
        mtr_rf.setVelocity(TPS * power_rf);
        mtr_lr.setVelocity(TPS * power_lr);
        mtr_lf.setVelocity(TPS * power_lf);
    }

    public void setSpeed(double forward, double sideways, double rotate) {
        // OK, so the maximum-minimum is the sum of the absolute values of forward, side, and turn
        SCALE = 1;
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

    public void setTPS(int TPS) {
        if (TPS > 2800) { MecanumBase.TPS = 2800; }
        else if (TPS < 0) { MecanumBase.TPS = 0; }
        else { MecanumBase.TPS = TPS; }
    }

    public void addTPS(int amount) { setTPS(TPS + amount); }

    public DriveMode getDriveMode() { return driveMode; }

    public void setDriveMode(DriveMode driveMode) { this.driveMode = driveMode; }

    public DriveMode switchDriveMode() {
        switch (driveMode) {
            case FOURFINGER:
                setDriveMode(MecanumBase.DriveMode.TWOFINGER);
                break;
            case TWOFINGER:
                setDriveMode(MecanumBase.DriveMode.FOURFINGER);
                break;
        }
        return driveMode;
    }
}
