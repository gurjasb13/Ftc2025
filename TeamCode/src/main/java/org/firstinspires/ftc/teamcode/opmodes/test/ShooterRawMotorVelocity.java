package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Raw Velocity Test", group = "Test")
public class ShooterRawMotorVelocity extends OpMode {

    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;

    @Override
    public void init() {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {

        // Left stick controls power
        double power = -gamepad1.left_stick_y;

        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);

        double vel1 = shooterMotor1.getVelocity();  // ticks per second
        double vel2 = shooterMotor2.getVelocity();

        telemetry.addData("Power", power);
        telemetry.addData("Motor1 ticks/sec", vel1);
        telemetry.addData("Motor2 ticks/sec", vel2);
        telemetry.update();
    }
}
