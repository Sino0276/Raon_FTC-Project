package org.firstinspires.ftc.teamcode.commands.groups;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class TurretTrackingTagCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;

    public TurretTrackingTagCommand(TurretSubsystem turret, VisionSubsystem vision) {
        this.turret = turret;
        this.vision = vision;

        addRequirements(turret, vision);
    }

}
