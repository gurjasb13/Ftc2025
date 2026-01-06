package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class VisionAlignCommand extends CommandBase {
    private final DrivebaseSubsystem drivebase;
    private final Limelight3A limelight;
    private final PIDController pid;

    private LLResult latestResult;

    public VisionAlignCommand(DrivebaseSubsystem drivebase, Limelight3A limelight) {
        this.drivebase = drivebase;
        this.limelight = limelight;

        pid = new PIDController(0.09, 0, 0.0001);

        addRequirements(drivebase);
    }

    @Override
    public void execute() {
        latestResult = limelight.getLatestResult();
        if (latestResult == null || !latestResult.isValid()) {
            drivebase.drive(0,0,0);
            return;
        }

        double tx = latestResult.getTx();
        double strafe = pid.calculate(tx, 0);

        strafe = Math.max(-0.7, Math.min(0.7, strafe));
        drivebase.drive(-strafe, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.drive(0, 0, 0);
    }
}