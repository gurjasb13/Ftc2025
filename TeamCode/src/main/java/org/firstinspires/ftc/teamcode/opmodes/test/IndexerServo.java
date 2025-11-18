package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class IndexerServo extends OpMode {
    private CRServo servo;

    @Override
    public void init() {
        servo= hardwareMap.get(CRServo.class, "indexer");
    }

    @Override
    public void loop() {
        if (gamepad1.x){
            servo.setPower(1);
        }
        else if (gamepad1.b){
            servo.setPower(-1);
        }
        else{
            servo.setPower(0);
        }
    }
}
