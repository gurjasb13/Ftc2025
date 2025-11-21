package org.firstinspires.ftc.teamcode.opmodes.test;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class VisionDataTest extends OpMode {
    private Limelight3A limelight;
    private DrivebaseSubsystem drivebaseSubsystem;
    private PIDController strafePID;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        strafePID=new PIDController(0.09, 0.0, 0.00001);
    }


    @Override
    public void loop() {
        LLResult latestResult = limelight.getLatestResult();

        if (latestResult.isValid() && gamepad1.x) {
            double tx = latestResult.getTx();
            double strafePower = strafePID.calculate(tx, 0);

            // Clamp
            strafePower = Math.max(-0.7, Math.min(0.7, strafePower));

            drivebaseSubsystem.drive(-strafePower, 0, 0);

            telemetry.addData("Has Target", true);
            telemetry.addData("distance", tx);
            telemetry.addData("power", strafePower);
        }
        else if (gamepad1.x) {
            telemetry.addData("VISION", "No target detected");
        }

        else {
            telemetry.addData("Has target", false);
        }

        if (limelight.isConnected()) {
            telemetry.addData("connected", true);
        }
        telemetry.update();
    }
}
