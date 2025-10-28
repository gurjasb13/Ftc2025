package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class VisionDataTest extends OpMode {
    private Limelight3A limelight3A;

    @Override
    public void init() {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(8);
    }

    @Override
    public void loop() {
        double tx = limelight3A.getLatestResult().getTx();
        double ty = limelight3A.getLatestResult().getTy();
        double ta = limelight3A.getLatestResult().getTa();

        telemetry.addData("Tx", tx);
        telemetry.addData("Ty", ty);
        telemetry.addData("Ta", ta);
        telemetry.update();
    }
}
