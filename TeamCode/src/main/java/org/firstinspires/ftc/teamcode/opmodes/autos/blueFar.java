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

@Autonomous(name = "Blue Far Start", group = "Autonomous")
@Configurable
public class blueFar extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;
    private feeder feeder;

    private ElapsedTime timer = new ElapsedTime();

    // Shooter PD + feedforward
    private PDController shooterController = new PDController(0.15, 0.05);
    private double shooterTargetRPM = 0;
    private static final double MAX_RPM = 5700;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gate = new Gate(hardwareMap);
        feeder = new feeder(hardwareMap);

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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.344, 8.934), new Pose(59.344, 20.207))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(123))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.344, 20.207), new Pose(48.071, 35.734))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.071, 35.734), new Pose(21.908, 35.309))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.908, 35.309), new Pose(59.131, 22.121))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(123))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.131, 22.121), new Pose(47.645, 58.919))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.645, 58.919), new Pose(22.547, 59.131))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(22.547, 59.131), new Pose(60.620, 22.121))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(123))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60.620, 22.121), new Pose(55.516, 51.261))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(123), Math.toRadians(123))
                    .build();
        }
    }
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1);

                shooterTargetRPM = 3800;
                gate.setPosition(0.4);
                feeder.setPosition(0.1);

                timer.reset();
                pathState = 1;
                break;

            case 1:
                if (timer.seconds() >= 4.5) {
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 6.5&& !follower.isBusy()) {
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    shooterTargetRPM = 0;
                    follower.followPath(paths.Path2);
                    pathState=3;
                    break;
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.Path3);
                    pathState=4;
                    break;
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.Path4);
                    intakeSubsystem.setPower(0);
                    shooterTargetRPM=3600;
                    timer.reset();
                    pathState=5;
                    break;
                }
                break;
            case 5:
                if (!follower.isBusy()&&timer.seconds()>4.5){
                    intakeSubsystem.setPower(1);
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState=6;
                    break;
                }
                break;
            case 6:
                if (!follower.isBusy() && timer.seconds()>6.5){
                    shooterSubsystem.runToRPM(0);
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    follower.followPath(paths.Path5);
                    pathState=7;
                    break;
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    follower.setMaxPower(0.4);
                    follower.followPath(paths.Path6);
                    pathState=8;
                    break;
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    intakeSubsystem.setPower(0);
                    timer.reset();
                    follower.setMaxPower(1);
                    follower.followPath(paths.Path7);
                    pathState=9;
                    break;
                }
                break;

            case 9:
                if (!follower.isBusy()&& timer.seconds()>4.5){
                    intakeSubsystem.setPower(1.0);
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState=10;
                    break;
                }
                break;
            case 10:
                if (timer.seconds()>7.0 && !follower.isBusy()){
                    gate.setPosition(0);
                    feeder.setPosition(0.1);
                    intakeSubsystem.setPower(0);
                    shooterSubsystem.setPower(0);
                    follower.followPath(paths.Path8);
                    break;
                }
                break;
        }

        return pathState;
    }
}
