package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Shooter Power Test", group = "Test")
@Config
public class ShooterTest extends LinearOpMode {
    private DcMotor shooterMotor1;
    private DcMotor shooterMotor2;

    public static double shooterPower = 0.5;// start with 50% power

    @Override
    public void runOpMode() throws InterruptedException{
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");

        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.right_stick_button==true) {
                shooterMotor1.setPower(shooterPower);
                shooterMotor2.setPower(shooterPower);
                telemetry.addLine("pressed");
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
            }

            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Button", gamepad1.right_stick_button);
            telemetry.addData("Power", shooterMotor1.getPower());
            telemetry.update();
        }
    }
}
