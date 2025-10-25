package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driving", group = "Ready")
public class Driving extends OpMode {

    private MecanumBase drive;
    private ShooterBase shooter;
    private PusherBase pusher;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet = new TelemetryPacket();
    private double forward, sideways, rotate;

    @Override
    public void init() {
        drive = new MecanumBase(hardwareMap);
        shooter = new ShooterBase(hardwareMap);
        pusher = new PusherBase(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) { drive.switchDriveMode(); }
        else if (gamepad1.dpadUpWasPressed()) { drive.addTPS(700);}
        else if (gamepad1.dpadDownWasPressed()) { drive.addTPS(-700);}

        switch (drive.getDriveMode()) {
            case FOURFINGER:
                forward = gamepad1.left_stick_y;
                sideways = gamepad1.right_stick_x;
                rotate = gamepad1.left_trigger - gamepad1.right_trigger;
                break;

            case TWOFINGER:
                forward = gamepad1.left_stick_y;
                sideways = gamepad1.left_stick_x;
                rotate = -gamepad1.right_stick_x;
                break;
        }
        drive.setSpeed(forward, sideways, rotate);


        // gamepad2 (Shooter) =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
        if (gamepad2.aWasPressed()) {
//            shooter.switchState();
            switch (shooter.getState()) {
                case ACTIVE:
                    shooter.setState(ShooterBase.State.UNACTIVE);
                    shooter.setPower(0);
                    break;

                case UNACTIVE:
                    shooter.setState(ShooterBase.State.ACTIVE);
                    shooter.setVelocity(shooter.TPS);
                    break;
            }
        }
        else if (gamepad2.dpadUpWasPressed()) { shooter.addTPS(100);}
        else if (gamepad2.dpadDownWasPressed()) { shooter.addTPS(-100);}

        else if (gamepad2.bWasPressed()) {
            switch (pusher.getState()) {
                case ACTIVE:
                    pusher.setState(PusherBase.State.UNACTIVE);
                    pusher.moveMaxPosition();     // 여기
                    break;

                case UNACTIVE:
                    pusher.setState(PusherBase.State.ACTIVE);
                    pusher.moveMinPosition();        // 여기
                    break;
            }
        }


        showData();
    }

        private void showData() {
        packet.put("DriveMode", drive.getDriveMode());
        packet.put("TPS(Shooter)", shooter.TPS);
        packet.put("TPS(Drive)", drive.TPS);
        packet.put("forward", forward);
        packet.put("sideways", sideways);
        packet.put("rotate", rotate);
        dashboard.sendTelemetryPacket(packet);
    }
}