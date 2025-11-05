package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@TeleOp(name = "Shooter PD tuning")
@Config
public class ShooterPDtuning extends LinearOpMode {
    ShooterSubsystem shooterSubsystem;
    private PDController controller;

    public static double kP = 0.0005;
    public static double kD = 0.0001;

    private double targetRPM = 0;


    @Override
    public void runOpMode() throws InterruptedException{
        shooterSubsystem= new ShooterSubsystem(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            controller.setP(kP);
            controller.setD(kD);

            double power = controller.calculate(shooterSubsystem.getCurrentRPM(), targetRPM);
            power = Math.max(0, Math.min(power, 1));

            if (gamepad1.x==true) {
                shooterSubsystem.setPower(power);
                telemetry.addLine("pressed");
            } else {
                shooterSubsystem.stop();
            }

            telemetry.addData("Shooter Power", power);
            telemetry.addData("Button", gamepad1.x);
            telemetry.update();
        }
    }
}
