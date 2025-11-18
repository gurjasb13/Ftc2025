package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "DualServoGate")
public class DualServoGate extends LinearOpMode {

    private Servo servo1;

    public static double kickposition= 0.7;
    public static double restposition= 0;
    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(Servo.class, "gateservo");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                servo1.setPosition(kickposition);
            }
            else{
                servo1.setPosition(restposition);
            }

            telemetry.addData("Servo on/off", servo1.getPosition());
            telemetry.update();
        }
    }
}
