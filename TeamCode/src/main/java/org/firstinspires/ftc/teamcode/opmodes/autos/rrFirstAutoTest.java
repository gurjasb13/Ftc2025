package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.intake.IntakeAction;
import org.firstinspires.ftc.teamcode.roadrunner.ActionCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.Set;

@Autonomous(name = "RR path")
public class rrFirstAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(55.516, 136.130, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .splineTo(new Vector2d(46.369, 91.037), Math.toRadians(270))
                        .splineTo(new Vector2d(82.529, 54.665), Math.toRadians(130))
                        .turnTo(Math.toRadians(130)) // replaces setLinearHeadingInterpolation
                        .build()
        );
    }
}