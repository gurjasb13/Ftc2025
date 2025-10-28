package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class VisionSubsystem extends SubsystemBase {
    private Limelight3A limelight3A;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public void setPipeline(int pipeline) {
        limelight3A.pipelineSwitch(pipeline);
    }
}
