package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.drivebase.AutoAlignDrivebaseVision;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class AutoAlignDrivebase extends OpMode {
    DrivebaseSubsystem drivebaseSubsystem;
    VisionSubsystem visionSubsystem;

    AutoAlignDrivebaseVision autoAlignDrivebaseVision;

    @Override
    public void init() {
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        visionSubsystem = new VisionSubsystem(hardwareMap);

        autoAlignDrivebaseVision = new AutoAlignDrivebaseVision(drivebaseSubsystem, visionSubsystem, gamepad1);

    }

    @Override
    public void loop() {
        autoAlignDrivebaseVision.execute();

        telemetry.addData("Has Target", visionSubsystem.hasTarget());
        telemetry.addData("Tx", visionSubsystem.getTx());
    }
}
