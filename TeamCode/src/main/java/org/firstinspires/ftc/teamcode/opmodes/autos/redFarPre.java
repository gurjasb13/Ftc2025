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

import com.arcrobotics.ftclib.controller.PDController;

@Autonomous(name = "Red Far Start Pre", group = "Autonomous")
@Configurable
public class redFarPre extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;

    private ElapsedTime timer = new ElapsedTime();

    // Shooter PD + feedforward
    private PDController shooterController = new PDController(0.15, 0.05);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(89, 9, Math.toRadians(90)));

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
                                    new Pose(88.000, 9.000),

                                    new Pose(88.908, 16.516)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.908, 16.516),

                                    new Pose(88.815, 35.631)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.815, 35.631),

                                    new Pose(124.933, 35.376)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(124.933, 35.376),

                                    new Pose(88.809, 16.274)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(63))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.809, 16.274),

                                    new Pose(94.563, 13.992)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.563, 13.992),

                                    new Pose(126.992, 13.168)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.992, 13.168),

                                    new Pose(115.023, 17.281)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(115.023, 17.281),

                                    new Pose(131.249, 13.810)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.249, 13.810),

                                    new Pose(117.205, 17.036)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117.205, 17.036),

                                    new Pose(131.294, 13.547)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.294, 13.547),

                                    new Pose(88.837, 16.574)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(67))

                    .build();

            Path12 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.837, 16.574),

                                    new Pose(89.560, 63.742)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(63))

                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);

                shooterTargetRPM = 5300;
                gate.setPosition(0.9);
                pathState = 1;
                break;

            case 1:
                if ((Math.abs(shooterSubsystem.getCurrentRPM() - 5300) <= 150)) {
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
                    shooterTargetRPM=5300;
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path11);
                    pathState=8;
                    break;
                }
                break;

            case 8:
                if (!follower.isBusy()&&(Math.abs(shooterSubsystem.getCurrentRPM() - 5300) <= 50)){
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
                    shooterSubsystem.setPower(0);
                    follower.followPath(paths.Path6);
                    break;
                }
                break;
        }

        return pathState;
    }
}