package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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

    private ElapsedTime timer = new ElapsedTime();

    // Continuous PD shooter control
    private PDController shooterController = new PDController(0.15, 0.01);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;
    private static final double RPM_TOLERANCE = 50;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(121, 126, Math.toRadians(36)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gate = new Gate(hardwareMap);
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
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(121.000, 125.000),

                                    new Pose(87.329, 98.939)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.329, 98.939),

                                    new Pose(87.208, 85.294)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.208, 85.294),

                                    new Pose(123.624, 84.208)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.624, 84.208),

                                    new Pose(87.307, 99.151)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(34))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.307, 99.151),

                                    new Pose(85.932, 58.493)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.932, 58.493),

                                    new Pose(125.528, 58.505)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125.528, 58.505),
                                    new Pose(65.087, 54.239),
                                    new Pose(87.307, 98.927)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(36))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.307, 98.927),

                                    new Pose(85.428, 35.131)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.428, 35.131),

                                    new Pose(126.507, 34.944)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.507, 34.944),

                                    new Pose(87.166, 99.082)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(36))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.166, 99.082),

                                    new Pose(85.874, 58.350)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                shooterTargetRPM = 4000;
                gate.setPosition(0.9);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy() && Math.abs(shooterSubsystem.getCurrentRPM() - 4000) <= 50) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 3.0) {
                    shooterSubsystem.setPower(0);
                    gate.setPosition(0.9);
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 4000;
                    follower.setMaxPower(1.0);
                    intakeSubsystem.setPower(0.5);
                    follower.followPath(paths.Path4);
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() && (Math.abs(shooterSubsystem.getCurrentRPM() - 4000) <= 50)) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if (timer.seconds() > 3.0) {
                    follower.followPath(paths.Path5);
                    shooterSubsystem.setPower(0);
                    gate.setPosition(0.9);
                    pathState = 7;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 4000;
                    follower.setMaxPower(1.0);
                    intakeSubsystem.setPower(0.5);
                    follower.followPath(paths.Path7);
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy() && (Math.abs(shooterSubsystem.getCurrentRPM() - 4000) <= 50)) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if (timer.seconds() > 3.0) {
                    follower.followPath(paths.Path8);
                    shooterSubsystem.setPower(0);
                    gate.setPosition(0.9);
                    pathState = 11;
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path9);
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 4000;
                    follower.setMaxPower(1.0);
                    intakeSubsystem.setPower(0.5);
                    follower.followPath(paths.Path10);
                    pathState = 13;
                }
                break;

            case 13:
                if (!follower.isBusy() && (Math.abs(shooterSubsystem.getCurrentRPM() - 4000) <= 50)) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState = 14;
                }
                break;

            case 14:
                if (timer.seconds() > 3.0) {
                    follower.followPath(paths.Path11);
                    shooterSubsystem.setPower(0);
                    intakeSubsystem.setPower(0);
                    gate.setPosition(0.9);
                }
                break;
        }

        return pathState;
    }
}
