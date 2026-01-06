package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Motor Direction Test", group = "Test")
public class ShooterMotorDirectionTest extends OpMode {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;

    @Override
    public void init() {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    @Override
    public void loop() {
        double power = -gamepad1.left_stick_y * 0.3; // capped power

        if (gamepad1.a) {
            shooterMotor1.setPower(power);
            shooterMotor2.setPower(0);
        }
        else if (gamepad1.b) {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(power);
        }
        else {
            shooterMotor1.setPower(0);
            shooterMotor2.setPower(0);
        }

        telemetry.addData("Testing Motor", gamepad1.a ? "Motor 1" :
                gamepad1.b ? "Motor 2" : "None");
        telemetry.addData("Power", power);
        telemetry.update();
    }
}
