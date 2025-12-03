package org.firstinspires.ftc.teamcode.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder;

@Config
@TeleOp
public class VisionDataTest extends OpMode {
    private Limelight3A limelight;

    public static double targetRPM;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Gate gate;
    feeder feeder;
    IntakeCommand intakeCommand;
    ShootRPM shootRPM;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        shooterSubsystem= new ShooterSubsystem(hardwareMap);
        intakeSubsystem= new IntakeSubsystem(hardwareMap);
        gate= new Gate(hardwareMap);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad2);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);

    }


    @Override
    public void loop() {
        LLResult latestResult = limelight.getLatestResult();

        if (latestResult.isValid()) {
            double ty = latestResult.getTy();

            telemetry.addData("Has Target", true);
            telemetry.addData("distance", ty);
        }

        else {
            telemetry.addData("Has target", false);
        }

        intakeCommand.execute();

        if (gamepad2.y){
            shooterSubsystem.runToRPM(targetRPM);
        } else{
            shooterSubsystem.setPower(0);
        }

        if (gamepad2.right_bumper) {
            gate.setPosition(0);
        }else{
            gate.setPosition(0.4);
        }

        telemetry.addData("Current RPM", shooterSubsystem.getCurrentRPM());
        telemetry.update();
    }
}
