package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.shootPower;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@TeleOp
public class WorkingTele extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    private shootPower shootPower;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    @Override
    public void init() {
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shootPower = new shootPower(shooterSubsystem, gamepad1);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad1);

        drivebaseSubsystem= new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand= new RobotCentricCommand(drivebaseSubsystem, gamepad1);
    }

    @Override
    public void loop() {
        shootPower.execute();
        intakeCommand.execute();
        robotCentricCommand.execute();
    }

    @Override
    public void stop() {
        shootPower.end();
        intakeCommand.end();
    }
}