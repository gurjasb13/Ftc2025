package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class ChatAlign extends CommandBase {
    private final DrivebaseSubsystem drivebaseSubsystem;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    // Limelight objects now inside the command
    public Limelight3A limelight;
    public LLResult latestResult;

    private final PIDController strafePID;
    private final double targetDistance = 40.0;

    public ChatAlign(
            DrivebaseSubsystem drivebaseSubsystem,
            Gamepad gamepad,
            Telemetry telemetry,
            HardwareMap hardwareMap
    ) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        // Init Limelight here
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        strafePID = new PIDController(0.0009, 0, 0.00001);

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
        // Always pull newest Limelight packet
        latestResult = limelight.getLatestResult();

        // Debug telemetry
        telemetry.addData("Has Target", hasTarget());
        telemetry.addData("Tx", getTx());
        telemetry.addData("Ty", getTy());
        telemetry.update();

        // Only align if X button held + target detected
        if (gamepad.x && hasTarget()) {

            double tx = getTx();
            double strafePower = strafePID.calculate(tx, 0);

            // Clamp
            strafePower = Math.max(-0.4, Math.min(0.4, strafePower));

            drivebaseSubsystem.drive(0, strafePower, 0);
        }
        else if (gamepad.x) {
            telemetry.addData("VISION", "No target detected");
        }
    }

    // -----------------------------
    // Vision helper functions
    // -----------------------------

    private boolean hasTarget() {
        return latestResult != null;
    }

    private double getTx() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTx();
    }

    private double getTy() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTy();
    }

    @Override
    public boolean isFinished() {
        return hasTarget() &&
                Math.abs(getTx()) < 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.stop();
    }
}
