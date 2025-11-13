package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
@Config
@TeleOp(name = "DualServoGate")
public class DualServoGate extends LinearOpMode {

    private CRServo servo1;
    private CRServo servo2;
    @Override
    public void runOpMode() {
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.b) {
                servo1.setPower(1.0);
                servo2.setPower(1.0);
            }

            if (gamepad1.a) {
                servo1.setPower(0);
                servo2.setPower(0);
            }
            telemetry.addData("Servo on/off", servo1.getPower());
            telemetry.update();
        }
    }
}
