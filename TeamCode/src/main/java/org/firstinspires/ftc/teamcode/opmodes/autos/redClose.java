package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.feeder;
import com.arcrobotics.ftclib.controller.PDController;

@Autonomous(name = "Red Close Start", group = "Autonomous")
@Configurable
public class redClose extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;
    private feeder feeder;

    private ElapsedTime timer = new ElapsedTime();

    // Continuous PD shooter control
    private PDController shooterController = new PDController(0.15, 0.05);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;
    private static final double RPM_TOLERANCE = 50;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121.453, 126.133, Math.toRadians(25)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gate = new Gate(hardwareMap);
        feeder = new feeder(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        updateShooterRPM(); // continuous control

        telemetry.addData("shooter rpm", shooterSubsystem.getCurrentRPM());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }

    private void updateShooterRPM() {
        double currentRPM = shooterSubsystem.getCurrentRPM();

        // feedforward + PD
        double feedforward = shooterTargetRPM / MAX_RPM;
        double pdOutput = shooterController.calculate(currentRPM, shooterTargetRPM);
        double power = feedforward + pdOutput;

        power = Math.max(0, Math.min(power, 1));
        shooterSubsystem.setPower(power);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(121.453, 126.133), new Pose(97.631, 106.777)))
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36)).build();
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(97.631, 106.777), new Pose(97.418, 86.783)))
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(180)).build();
            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(97.418, 86.783), new Pose(110.818, 86.783)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path4 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(110.818, 86.783), new Pose(98.056, 106.990)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(36)).build();
            Path5 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(98.056, 106.990), new Pose(98.482, 62.109)))
                    .setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(180)).build();
            Path6 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(98.482, 62.109), new Pose(112.095, 61.897)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180)).build();
            Path7 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(112.095, 61.897), new Pose(98.056, 107.415)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(36)).build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                shooterTargetRPM = 2700;
                gate.setPosition(0.4);
                feeder.setPosition(0.1);
                timer.reset();
                pathState = 1;
                break;

            case 1:
                if (timer.seconds() >= 2.5) intakeSubsystem.setPower(1);
                if (timer.seconds() >= 3.5) {
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 4.5) {
                    shooterTargetRPM = 0;
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 2700;
                    follower.followPath(paths.Path4);
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() && timer.seconds() > 3.0) {
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    timer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (timer.seconds() > 2) {
                    follower.followPath(paths.Path5);
                    shooterTargetRPM = 0;
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 2700;
                    follower.followPath(paths.Path7);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    timer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if (timer.seconds() > 2) {
                    shooterTargetRPM = 0;
                    intakeSubsystem.setPower(0);
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                }
                break;
        }
        return pathState;
    }

}
