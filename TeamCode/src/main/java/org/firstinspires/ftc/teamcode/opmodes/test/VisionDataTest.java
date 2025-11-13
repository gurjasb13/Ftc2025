package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

@TeleOp
public class VisionDataTest extends OpMode {
    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }


    @Override
    public void loop() {
        LLResult latestResult = limelight.getLatestResult();

        if (!latestResult.isValid()) {
            telemetry.addLine("No result object yet.");
        } else if (latestResult.isValid()) {
            telemetry.addData("Has target", true);
            telemetry.addData("Target Type", latestResult.getFiducialResults());
            telemetry.addData("X Offset", latestResult.getTx());
            telemetry.addData("Y Offset", latestResult.getTy());
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
