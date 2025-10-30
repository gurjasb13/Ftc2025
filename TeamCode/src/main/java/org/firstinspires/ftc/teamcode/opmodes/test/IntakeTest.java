package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
public class IntakeTest extends OpMode {
    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    private IntakeCommand intakeCommand;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void init() {
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad1);

        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);
    }

    @Override
    public void loop() {
        intakeCommand.execute();
        robotCentricCommand.execute();
    }

    @Override
    public void stop() {
        intakeCommand.end();
    }
}
