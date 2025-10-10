package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

@Autonomous(name = "RoadRunner Simple Auto")
public class rrFirstAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(initialPose)
                        .lineToX(64)
                        .lineToX(0)
                        .splineTo(new Vector2d(48, 48), Math.toRadians(0))
                        .build()
        );
    }
}
