package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gate extends SubsystemBase {
    private Servo gate;

    public Gate(HardwareMap hardwareMap){
        gate = hardwareMap.get(Servo.class, "gate");
    }

    public void setPosition(double position){
        gate.setPosition(position);
    }
}
