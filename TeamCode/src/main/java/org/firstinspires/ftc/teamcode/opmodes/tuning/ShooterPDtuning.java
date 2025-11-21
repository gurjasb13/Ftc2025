package org.firstinspires.ftc.teamcode.opmodes.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PDController;
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
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    DrivebaseSubsystem drivebaseSubsystem;
    Gate gate;
    feeder feeder;

    IntakeCommand intakeCommand;
    RobotCentricCommand robotCentricCommand;
    ShootRPM shootRPM;


    @Override
    public void init(){
        shooterSubsystem= new ShooterSubsystem(hardwareMap);
        intakeSubsystem= new IntakeSubsystem(hardwareMap);
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        gate= new Gate(hardwareMap);
        feeder= new feeder(hardwareMap);

        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad2);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);
    }

    @Override
    public void loop(){
        intakeCommand.execute();
        shootRPM.execute();
        robotCentricCommand.execute();

        if (gamepad2.right_bumper) {
            gate.setPosition(0);
        }else{
            gate.setPosition(0.4);
        }

        if (gamepad2.left_bumper){
            feeder.setPosition(0.3);
        }else{
            feeder.setPosition(0.1);
        }

        telemetry.addData("Current RPM", shooterSubsystem.getCurrentRPM());
        telemetry.addData("Button", gamepad1.x);
        telemetry.update();
    }
}
