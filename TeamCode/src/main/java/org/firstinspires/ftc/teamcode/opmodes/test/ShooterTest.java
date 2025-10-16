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
    private DcMotor shooterMotor3;

    public static double shooterPower = 1.0;


    @Override
    public void runOpMode() throws InterruptedException{
        shooterMotor1 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        shooterMotor3 = hardwareMap.get(DcMotor.class, "shooterMotor3");

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        shooterMotor3.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.right_stick_button==true) {
                shooterMotor1.setPower(shooterPower);
                shooterMotor2.setPower(shooterPower);
                shooterMotor3.setPower(shooterPower);
                telemetry.addLine("pressed");
            } else {
                shooterMotor1.setPower(0);
                shooterMotor2.setPower(0);
                shooterMotor3.setPower(0);
            }

            telemetry.addData("Shooter Power", shooterPower);
            telemetry.addData("Button", gamepad1.right_stick_button);
            telemetry.update();
        }
    }
}
