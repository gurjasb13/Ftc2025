package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivebase.FieldCentricCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

@TeleOp
public class FieldCentricDrive extends OpMode {
    private DrivebaseSubsystem drivebaseSubsystem;
    private FieldCentricCommand fieldCentricCommand;

    public void init() {
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        fieldCentricCommand = new FieldCentricCommand(drivebaseSubsystem, gamepad1);

        telemetry.addLine("IMU Initialized. Waiting for start...");
        telemetry.update();
    }

    public void loop() {
        fieldCentricCommand.execute();

        telemetry.addData("Heading (rad)", drivebaseSubsystem.getHeading());
        telemetry.addData("Joystick (x, y, rx)", "%.2f, %.2f, %.2f", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        telemetry.update();
    }
}
