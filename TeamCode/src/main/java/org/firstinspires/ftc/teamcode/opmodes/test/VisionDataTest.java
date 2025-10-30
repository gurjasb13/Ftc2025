package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class VisionDataTest extends OpMode {
    private VisionSubsystem visionSubsystem;

    @Override
    public void init() {
        visionSubsystem = new VisionSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Has Target", visionSubsystem.hasTarget());
        telemetry.addData("Tx", visionSubsystem.getTx());
        telemetry.addData("Ty", visionSubsystem.getTy());
        telemetry.addData("Distance", visionSubsystem.getDistance());
    }
}
