package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class DrivebasePIDtuning extends OpMode {
    private DcMotor rfmotor, lfmotor, rbmotor, lbmotor;

    public static double strafekP = 0.01;
    public static double strafekI = 0.0;
    public static double strafekD = 0.0;

    public static double targetPosition = 1000; // encoder ticks (adjust as needed)

    private PIDController strafePID = new PIDController(strafekP, strafekI, strafekD);

    @Override
    public void init() {
        rfmotor = hardwareMap.get(DcMotor.class, "rf");
        lfmotor = hardwareMap.get(DcMotor.class, "lf");
        rbmotor = hardwareMap.get(DcMotor.class, "rb");
        lbmotor = hardwareMap.get(DcMotor.class, "lb");

        rbmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rfmotor.setDirection(DcMotor.Direction.REVERSE);
        rbmotor.setDirection(DcMotor.Direction.REVERSE);

        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        strafePID.setPID(strafekP, strafekI, strafekD);

        if (gamepad1.y) {
            double currentPosition = rbmotor.getCurrentPosition();
            double strafePower = strafePID.calculate(currentPosition, targetPosition);
            strafePower = Math.max(-0.5, Math.min(0.5, strafePower));

            // Strafing motor power directions
            lfmotor.setPower(strafePower);
            rfmotor.setPower(-strafePower);
            lbmotor.setPower(-strafePower);
            rbmotor.setPower(strafePower);
        } else {
            lfmotor.setPower(0);
            rfmotor.setPower(0);
            lbmotor.setPower(0);
            rbmotor.setPower(0);
        }

        telemetry.addData("Current Pos", rbmotor.getCurrentPosition());
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("P", strafekP);
        telemetry.addData("I", strafekI);
        telemetry.addData("D", strafekD);
        telemetry.update();
    }
}
