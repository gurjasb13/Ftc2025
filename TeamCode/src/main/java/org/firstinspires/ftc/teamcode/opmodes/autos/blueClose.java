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

@Autonomous(name = "Blue Close Start", group = "Autonomous")
@Configurable // Panels
public class blueClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private Gate gate;
    private feeder feeder;

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(24, 128, Math.toRadians(143)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        gate = new Gate(hardwareMap);
        feeder = new feeder(hardwareMap);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.185, 126.558), new Pose(43.391, 110.606))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(143))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.391, 110.606), new Pose(43.391, 85.294))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(43.391, 85.294), new Pose(26.801, 85.294))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(26.801, 85.294), new Pose(42.966, 109.968))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.966, 109.968), new Pose(42.966, 60.833))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(42.966, 60.833), new Pose(25.737, 59.770))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.737, 59.770), new Pose(43.179, 110.606))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(143))
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                shooterSubsystem.setPower(1.0);
                gate.setPosition(0.4);
                feeder.setPosition(0.1);

                timer.reset();
                pathState = 1;
                break;

            case 1:
                if (timer.seconds() >= 3.5) {
                    intakeSubsystem.setPower(1);
                }

                if (timer.seconds() >= 4.5) {
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    pathState = 2;
                }
                break;

            case 2:
                if (timer.seconds() > 5.5) {
                    shooterSubsystem.setPower(1.0);
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    follower.followPath(paths.Path2);
                    pathState = 3;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    // **Slow Path3**
                    follower.setMaxPower(0.5);   // SLOW
                    follower.followPath(paths.Path3);
                    pathState = 4;
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    shooterSubsystem.setPower(1.0);

                    // Restore normal speed AFTER Path3
                    follower.setMaxPower(1.0);

                    follower.followPath(paths.Path4);
                    timer.reset();
                    pathState = 5;
                }
                break;

            case 5:
                if(!follower.isBusy() && timer.seconds() > 3.0){
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    timer.reset();
                    pathState = 6;
                }
                break;

            case 6:
                if(timer.seconds() > 2){
                    follower.followPath(paths.Path5);
                    shooterSubsystem.setPower(0);
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                    pathState = 7;
                }
                break;

            case 7:
                if(!follower.isBusy()){
                    // **Slow Path6**
                    follower.setMaxPower(0.5);   // SLOW
                    follower.followPath(paths.Path6);
                    pathState = 8;
                }
                break;

            case 8:
                if(!follower.isBusy()){
                    // Restore normal speed AFTER Path6
                    follower.setMaxPower(1.0);

                    shooterSubsystem.setPower(1.0);
                    follower.followPath(paths.Path7);
                    pathState = 9;
                }
                break;

            case 9:
                if(!follower.isBusy()){
                    gate.setPosition(0);
                    feeder.setPosition(0.4);
                    timer.reset();
                    pathState = 10;
                }
                break;

            case 10:
                if(timer.seconds() > 2){
                    shooterSubsystem.setPower(0);
                    intakeSubsystem.setPower(0);
                    gate.setPosition(0.4);
                    feeder.setPosition(0.1);
                }
                break;
        }
        return pathState;
    }

}
