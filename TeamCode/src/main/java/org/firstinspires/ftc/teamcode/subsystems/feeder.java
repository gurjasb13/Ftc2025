package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class feeder extends SubsystemBase {
    private Servo feeder;

    public feeder(HardwareMap hardwareMap){
        feeder = hardwareMap.get(Servo.class, "feeder");
    }

    public void setPosition(double position){
        feeder.setPosition(position);
    }
}
