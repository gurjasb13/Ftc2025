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
        follower.setStartingPose(new Pose(24, 127, Math.toRadians(143)));

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

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.248, 126.558), new Pose(43.391, 110.606))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.391, 110.606), new Pose(56.579, 87.208))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.579, 87.208), new Pose(25.950, 86.570))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.950, 86.570), new Pose(42.966, 109.968))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.966, 109.968), new Pose(59.982, 62.747))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.982, 62.747), new Pose(26.588, 61.897))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(26.588, 61.897), new Pose(43.179, 110.606))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.179, 110.606), new Pose(42.328, 55.728))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                    .build();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);
                shooterTargetRPM = 3100;
                gate.setPosition(0.4);
                feeder.setPosition(0.1);
                timer.reset();
                pathState = 1;
                break;

            case 1:
                if (timer.seconds() >= 5.0) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 6.5) {
                    shooterTargetRPM = 0;
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 3100;
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.Path4);
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if (!follower.isBusy() && timer.seconds() > 4.0) {
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
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    shooterTargetRPM = 3200;
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.Path7);
                    timer.reset();
                    pathState = 9;
                }
                break;

            case 9:
                if (!follower.isBusy()&&timer.seconds()>4.0) {
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    timer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if (!follower.isBusy() && timer.seconds()>2){
                    intakeSubsystem.setPower(0);
                    shooterSubsystem.setPower(0);
                    follower.followPath(paths.Path8);
                }
        }
        return pathState;
    }

}
