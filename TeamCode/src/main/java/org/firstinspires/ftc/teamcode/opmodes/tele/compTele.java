package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.LimelightShotCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp
public class compTele extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    private ShootRPM shootRPM;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    private Gate gate;
    private feeder feeder;

    private Limelight3A limelight;
    private LimelightShotCommand limelightShotCommand;

    @Override
    public void init() {

        CommandScheduler.getInstance().reset();

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand = new IntakeCommand(intakeSubsystem, gamepad2);

        gate = new Gate(hardwareMap);

        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        limelightShotCommand = new LimelightShotCommand(shooterSubsystem, limelight);
    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();

        if (gamepad2.right_bumper)
            gate.setPosition(0);
        else
            gate.setPosition(0.4);

        // run normal subsystem commands
        robotCentricCommand.execute();
        shootRPM.execute();
        intakeCommand.execute();

        if (gamepad2.y){
            limelightShotCommand.execute();
        }

        telemetry.addData("shooter rpm", shooterSubsystem.getCurrentRPM());
        telemetry.addData("distance", limelightShotCommand.getDistance(limelight.getLatestResult().getTy()));
        telemetry.update();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
