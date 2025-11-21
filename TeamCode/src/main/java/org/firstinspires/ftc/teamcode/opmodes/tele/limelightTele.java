package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRPM;
import org.firstinspires.ftc.teamcode.commands.shooter.Fire;
import org.firstinspires.ftc.teamcode.commands.drivebase.VisionAlignCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder;

import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp
public class limelightTele extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    private ShootRPM shootRPM;

    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;

    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    private Gate gate;
    private feeder feeder;

    private Limelight3A limelight;
    private LLResult result;
    private VisionAlignCommand visionAlignCommand;

    @Override
    public void init() {

        CommandScheduler.getInstance().reset();

        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        shootRPM = new ShootRPM(shooterSubsystem, gamepad2);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand = new IntakeCommand(intakeSubsystem, gamepad2);

        gate = new Gate(hardwareMap);
        feeder = new feeder(hardwareMap);

        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        visionAlignCommand = new VisionAlignCommand(drivebaseSubsystem, limelight);

        // default driving
        CommandScheduler.getInstance().setDefaultCommand(drivebaseSubsystem, robotCentricCommand);
    }

    @Override
    public void loop() {

        // Vision override
        if (gamepad1.x) {
            CommandScheduler.getInstance().schedule(visionAlignCommand);
        } else {
            CommandScheduler.getInstance().cancel(visionAlignCommand);
        }

        // Manual servos
        if (gamepad2.right_bumper) gate.setPosition(0);
        else gate.setPosition(0.4);

        if (gamepad2.left_bumper) feeder.setPosition(0.4);
        else feeder.setPosition(0.1);

        // NEW FIRE COMMAND
        if (gamepad2.x) {
            CommandScheduler.getInstance().schedule(
                    new Fire(shooterSubsystem, intakeSubsystem, gate, 3000)
            );
        }

        if (gamepad2.y) {
            CommandScheduler.getInstance().schedule(
                    new Fire(shooterSubsystem, intakeSubsystem, gate, 5000)
            );
        }

        // run default shooter + intake logic
        shootRPM.execute();
        intakeCommand.execute();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
