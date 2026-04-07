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

@Autonomous(name = "Blue Close Start", group = "Autonomous")
@Configurable
public class blueClose extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;

    private ElapsedTime timer = new ElapsedTime();

    // Continuous PD shooter control
    private PDController shooterController = new PDController(0.3, 0.05);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;
    private static final double RPM_TOLERANCE = 50;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(23, 126, Math.toRadians(143)));

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
                                    new Pose(23.000, 126.000),

                                    new Pose(54.365, 97.840)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.365, 97.840),

                                    new Pose(54.563, 84.824)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(139), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.563, 84.824),

                                    new Pose(21.202, 83.726)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(21.202, 83.726),

                                    new Pose(54.644, 97.642)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(139))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.644, 97.642),

                                    new Pose(59.131, 60.408)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(59.131, 60.408),

                                    new Pose(19.243, 59.770)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.243, 59.770),

                                    new Pose(54.376, 97.840)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(140))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.376, 97.840),

                                    new Pose(60.131, 36.462)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(138), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.131, 36.462),

                                    new Pose(19.932, 36.056)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.932, 36.056),

                                    new Pose(54.379, 97.826)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(139))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.379, 97.826),

                                    new Pose(60.148, 53.309)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(140))

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
                    intakeSubsystem.setPower(0.7);
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
                    intakeSubsystem.setPower(0.7);
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
                    follower.setMaxPower(0.7);
                    follower.followPath(paths.Path9);
                    pathState = 12;
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 4000;
                    follower.setMaxPower(1.0);
                    intakeSubsystem.setPower(0.7);
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
