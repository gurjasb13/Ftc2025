package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@TeleOp
public class PurplePrototypeTest extends OpMode {
    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    private ShooterSubsystem shooterSubsystem;
    private ShootRPM shootRPM;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    @Override
    public void init() {
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad1);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad1);
    }

    @Override
    public void loop() {
        robotCentricCommand.execute();
        shootRPM.execute();
        intakeCommand.execute();
    }

    @Override
    public void stop() {
        shootRPM.end();
        intakeCommand.end();
    }
}