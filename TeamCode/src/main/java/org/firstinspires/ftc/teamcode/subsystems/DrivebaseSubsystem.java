package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivebaseSubsystem extends SubsystemBase {
    private DcMotor rfmotor, lfmotor, rbmotor, lbmotor;

    public DrivebaseSubsystem(HardwareMap hardwareMap){

        rfmotor = hardwareMap.get(DcMotor.class, "rf");//Port3----right encoder
        lfmotor = hardwareMap.get(DcMotor.class, "lf");//Port2
        rbmotor = hardwareMap.get(DcMotor.class, "rb");//Port1-----perp
        lbmotor = hardwareMap.get(DcMotor.class, "lb");//Port0-----left encoder


        //set motor directions
        rfmotor.setDirection(DcMotor.Direction.REVERSE);
        lfmotor.setDirection(DcMotor.Direction.FORWARD);
        rbmotor.setDirection(DcMotor.Direction.REVERSE);
        lbmotor.setDirection(DcMotor.Direction.FORWARD);

        //set motor behaviours
        rfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private double cubicCurve(double input) {
        return 0.6 * Math.pow(input, 3) + 0.4 * input;
    }

    public void drive(double x, double y, double rx) {

        // Apply cubic scaling to soften low inputs
        x = cubicCurve(x);
        y = cubicCurve(y);
        rx = cubicCurve(rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        lfmotor.setPower(frontLeftPower);
        lbmotor.setPower(backLeftPower);
        rfmotor.setPower(frontRightPower);
        rbmotor.setPower(backRightPower);
    }

    public double getStrafePosition() {
        double lateral = rbmotor.getCurrentPosition(); // ticks

        double ticksPerInch = 2318.4;
        return lateral / ticksPerInch;
    }


    public void stop() {
        drive(0, 0, 0);
    }

}

