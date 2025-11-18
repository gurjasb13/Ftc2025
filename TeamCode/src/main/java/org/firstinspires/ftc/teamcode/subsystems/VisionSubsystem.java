package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private LLResult latestResult;

    private final double cameraHeight = 5;//change
    private final double targetHeight = 29;//change
    private final double cameraAngle = Math.toRadians(70);//change

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void periodic() {
        latestResult=limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid();
    }

    public double getTx() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTx(); // degrees
    }

    public double getTy() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTy(); // degrees
    }

    public double getDistance() {
        if (!hasTarget()) return 0.0;

        double tyRadians = Math.toRadians(getTy());
        return (targetHeight - cameraHeight) / Math.tan(cameraAngle + tyRadians);
    }

    public double getTargetArea() {
        if (!hasTarget()) return 0.0;
        return latestResult.getTa(); // normalized area of target
    }

    public void printTelemetry(Telemetry telemetry) {
        telemetry.addData("Has Target", hasTarget());
        telemetry.addData("tx", getTx());
        telemetry.addData("ty", getTy());
        telemetry.addData("Distance (cm)", getDistance());
        telemetry.addData("Target Area", getTargetArea());
        telemetry.update();
    }
}
