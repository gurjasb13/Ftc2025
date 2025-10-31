package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AutoAlignDrivebaseVision extends CommandBase {
    private final DrivebaseSubsystem drivebaseSubsystem;
    private final VisionSubsystem vision;
    private final Gamepad gamepad;

    private final PIDController strafePID;
    //private final PIDController distancePID;

    private final double targetDistance = 40.0;

    public AutoAlignDrivebaseVision(DrivebaseSubsystem drivebaseSubsystem, VisionSubsystem vision, Gamepad gamepad) {
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.vision = vision;
        this.gamepad=gamepad;

        strafePID = new PIDController(0.001, 0, 0.0001);
        //distancePID = new PIDController(0.04, 0, 0.001);

        addRequirements(drivebaseSubsystem);
    }

    @Override
    public void execute() {
        if (gamepad.x && vision.hasTarget()) {

            double tx = vision.getTx(); // side error
            //double distance = vision.getDistance(); // forward distance
            //double distanceError = targetDistance - distance;

            double strafePower = strafePID.calculate(tx, 0);
            //double forwardPower = distancePID.calculate(distanceError, 0);

            strafePower = Math.max(-0.4, Math.min(0.4, strafePower));
            //forwardPower = Math.max(-0.3, Math.min(0.3, forwardPower));

            drivebaseSubsystem.drive(0, strafePower, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return vision.hasTarget() &&
                Math.abs(vision.getTx()) < 1.0;
                //Math.abs(vision.getDistance() - targetDistance) < 3.0;
    }

    @Override
    public void end(boolean interrupted) {
        drivebaseSubsystem.stop();
    }
}

