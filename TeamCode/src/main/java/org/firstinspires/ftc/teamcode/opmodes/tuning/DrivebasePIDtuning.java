package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

@Config
@TeleOp
public class DrivebasePIDtuning extends OpMode {
    DrivebaseSubsystem drivebaseSubsystem;

    private double strafekP= 0;
    private double strafekI= 0;
    private double strafekD= 0;

    private double targetPosition = 24; // inches

    private PIDController strafePID = new PIDController(strafekP, strafekI, strafekD);

    @Override
    public void init() {
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            // Get current strafe position from encoders
            double currentPosition = drivebaseSubsystem.getStrafePosition(); // implement method in DriveSubsystem

            // Calculate PID output
            double strafePower = strafePID.calculate(currentPosition, targetPosition);

            // Clamp output
            strafePower = Math.max(-0.5, Math.min(0.5, strafePower));

            // Drive robot: forward=0, strafe=PID output, turn=0
            drivebaseSubsystem.drive(0, strafePower, 0);
        } else {
            // Stop strafing when button not pressed
            drivebaseSubsystem.drive(0, 0, 0);
        }

        // --- Telemetry for tuning ---
        telemetry.addData("Current Pos", drivebaseSubsystem.getStrafePosition());
        telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("P", strafePID.getP());
        telemetry.addData("I", strafePID.getI());
        telemetry.addData("D", strafePID.getD());
        telemetry.update();
    }
}

