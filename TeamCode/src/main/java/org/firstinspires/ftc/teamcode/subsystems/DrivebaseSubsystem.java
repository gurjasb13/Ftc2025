package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivebaseSubsystem extends SubsystemBase {
    private DcMotor rfmotor, lfmotor, rbmotor, lbmotor;

    public DrivebaseSubsystem(HardwareMap hardwareMap){

        rfmotor = hardwareMap.get(DcMotor.class, "rf");
        lfmotor = hardwareMap.get(DcMotor.class, "lf");
        rbmotor = hardwareMap.get(DcMotor.class, "rb");
        lbmotor = hardwareMap.get(DcMotor.class, "lb");

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

    public void drive(double x, double y, double rx){

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

}
