package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeAction;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name = "RR path")
public class rrFirstAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(72, 8, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        IntakeSubsystem intakeSubsystem = new IntakeSubsystem(hardwareMap);

        Action intakeAction = new IntakeAction(intakeSubsystem, 1.0, 2.0);

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineTo(new Vector2d(46.560, 24.035), Math.toRadians(90))
                        .afterTime(0.0, intakeAction)
                        .turnTo(Math.toRadians(130))
                        .build()
        );
    }
}