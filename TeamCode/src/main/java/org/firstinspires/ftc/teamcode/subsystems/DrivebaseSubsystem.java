package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DrivebaseSubsystem extends SubsystemBase {
    private DcMotor rfmotor, lfmotor, rbmotor, lbmotor;
    private BNO055IMU imu;

    public DrivebaseSubsystem(HardwareMap hardwareMap){

        rfmotor = hardwareMap.get(DcMotor.class, "rf");
        lfmotor = hardwareMap.get(DcMotor.class, "lf");
        rbmotor = hardwareMap.get(DcMotor.class, "rb");
        lbmotor = hardwareMap.get(DcMotor.class, "lb");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

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

    public void drive(double x, double y, double rx) {
        double botHeading = imu.getAngularOrientation().firstAngle;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        lfmotor.setPower(frontLeftPower);
        lbmotor.setPower(backLeftPower);
        rfmotor.setPower(frontRightPower);
        rbmotor.setPower(backRightPower);
    }

    public void resetIMU(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public double getHeading(){
        return imu.getAngularOrientation().firstAngle;
    }
}
