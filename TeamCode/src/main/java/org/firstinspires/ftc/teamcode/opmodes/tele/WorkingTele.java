package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;
import org.firstinspires.ftc.teamcode.commands.shooter.shootMode;
import org.firstinspires.ftc.teamcode.commands.shooter.shootPower;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;


@TeleOp
public class WorkingTele extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    private ShootRPM shootRPM;


    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private Gate gate;

    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;
    @Override
    public void init() {
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);

        gate = new Gate(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand= new IntakeCommand(intakeSubsystem, gamepad2);

        drivebaseSubsystem= new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand= new RobotCentricCommand(drivebaseSubsystem, gamepad1);
    }

    @Override
    public void loop() {
        shootRPM.execute();
        intakeCommand.execute();
        robotCentricCommand.execute();
    }

    @Override
    public void stop() {
        shootRPM.end();
        intakeCommand.end();
    }
}