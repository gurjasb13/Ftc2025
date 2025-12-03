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

@Autonomous(name = "Red Far Start", group = "Autonomous")
@Configurable
public class redFar extends OpMode {

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
        follower.setStartingPose(new Pose(84.8685376661743, 7.870014771048751, Math.toRadians(90)));

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
                            new BezierLine(new Pose(84.230, 8.934), new Pose(84.869, 20.207))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(63))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.869, 20.207), new Pose(95.716, 34.883))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.716, 34.883), new Pose(123.581, 35.521))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.581, 35.521), new Pose(87.634, 13.613))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(63))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.634, 13.613), new Pose(102.097, 57.855))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(63), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.097, 57.855), new Pose(124.219, 57.855))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124.219, 57.855), new Pose(87.421, 13.826))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(63))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(87.421, 13.826), new Pose(88.272, 37.010))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(63))
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
