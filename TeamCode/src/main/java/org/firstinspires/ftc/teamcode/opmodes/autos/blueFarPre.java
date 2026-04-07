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

@Autonomous(name = "Blue Far Pre", group = "Autonomous")
@Configurable
public class blueFarPre extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;

    private ElapsedTime timer = new ElapsedTime();

    // Shooter PD + feedforward
    private PDController shooterController = new PDController(0.35, 0.1);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 9, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gate = new Gate(hardwareMap);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        updateShooterRPM();

        telemetry.addData("shooterRPM", shooterSubsystem.getCurrentRPM());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.update(telemetry);
    }

    private void updateShooterRPM() {
        double currentRPM = shooterSubsystem.getCurrentRPM();

        double ff = shooterTargetRPM / MAX_RPM;     // Feedforward
        double pd = shooterController.calculate(currentRPM, shooterTargetRPM);

        double power = ff + pd;
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
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.900, 11.869),

                                    new Pose(54.880, 16.292)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(121))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.880, 16.292),

                                    new Pose(55.081, 36.496)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(121), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.081, 36.496),

                                    new Pose(13.109, 35.827)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(13.109, 35.827),

                                    new Pose(54.820, 15.907)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(116))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.820, 15.907),

                                    new Pose(46.221, 15.291)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(115), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(46.221, 15.291),

                                    new Pose(15.677, 12.317)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(121), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.677, 12.317),

                                    new Pose(26.986, 13.960)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-1))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.986, 13.960),

                                    new Pose(16.748, 12.406)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-1))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(16.748, 12.406),

                                    new Pose(26.900, 13.734)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(26.900, 13.734),

                                    new Pose(15.970, 12.379)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-1))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.970, 12.379),

                                    new Pose(54.868, 16.348)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(122))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.868, 16.348),

                                    new Pose(59.795, 46.806)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(119), Math.toRadians(118))

                    .build();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);

                shooterTargetRPM = 5150;
                gate.setPosition(0.9);
                pathState = 1;
                break;

            case 1:
                if ((Math.abs(shooterSubsystem.getCurrentRPM() - 5150) <= 150)) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 3&& !follower.isBusy()) {
                    gate.setPosition(0.9);
                    shooterSubsystem.setPower(0);
                    follower.followPath(paths.Path6);
                    pathState=3;
                    break;
                }
                break;

            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path7);
                    timer.reset();
                    pathState=4;
                    break;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path8);
                    timer.reset();
                    pathState=5;
                    break;
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path9);
                    timer.reset();
                    pathState=6;
                    break;
                }
                break;
            case 6:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.6);
                    follower.followPath(paths.Path10);
                    timer.reset();
                    pathState=7;
                    break;
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    intakeSubsystem.setPower(0.7);
                    shooterTargetRPM=5200;
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path11);
                    pathState=8;
                    break;
                }
                break;

            case 8:
                if (!follower.isBusy()&&(Math.abs(shooterSubsystem.getCurrentRPM() - 5200) <= 50)){
                    intakeSubsystem.setPower(1.0);
                    gate.setPosition(0.3);
                    timer.reset();
                    pathState=9;
                    break;
                }
                break;
            case 9:
                if (timer.seconds()>3 && !follower.isBusy()){
                    gate.setPosition(0.3);
                    intakeSubsystem.setPower(0);
                    shooterSubsystem.setPower(0);
                    follower.followPath(paths.Path6);
                    break;
                }
                break;
        }

        return pathState;
    }
}