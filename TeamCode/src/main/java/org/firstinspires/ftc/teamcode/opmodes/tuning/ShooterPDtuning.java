package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder;


@Config
@TeleOp
public class ShooterPDtuning extends OpMode {

    public static double targetRPM;

    private Limelight3A limelight3A;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    Gate gate;
    IntakeCommand intakeCommand;
    ShootRPM shootRPM;


    @Override
    public void init(){
        shooterSubsystem= new ShooterSubsystem(hardwareMap);
        intakeSubsystem= new IntakeSubsystem(hardwareMap);
        gate= new Gate(hardwareMap);

        limelight3A= hardwareMap.get(Limelight3A.class, "limelight");

        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad2);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);
    }

    @Override
    public void loop(){
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

        LLResult latestResult = limelight3A.getLatestResult();

        if (latestResult.isValid()) {
            double ty = latestResult.getTy();

            telemetry.addData("Has Target", true);
            telemetry.addData("distance", ty);
        }
        else{
            telemetry.addData("VISION", "No target detected");
        }

        telemetry.addData("Current RPM", shooterSubsystem.getCurrentRPM());
        telemetry.update();
    }
}
