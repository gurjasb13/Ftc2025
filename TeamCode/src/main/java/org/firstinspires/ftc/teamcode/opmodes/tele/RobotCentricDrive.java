package org.firstinspires.ftc.teamcode.opmodes.tele;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.commands.drivebase.RobotCentricCommand;
import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

@Disabled
@TeleOp
public class RobotCentricDrive extends OpMode {
    private DrivebaseSubsystem drivebaseSubsystem;
    private RobotCentricCommand robotCentricCommand;

    @Override
    public void init() {
        drivebaseSubsystem = new DrivebaseSubsystem(hardwareMap);
        robotCentricCommand = new RobotCentricCommand(drivebaseSubsystem, gamepad1);
    }

    @Override
    public void loop() {
        robotCentricCommand.execute();

        telemetry.addData("axial", gamepad1.left_stick_x);
        telemetry.addData("lateral", gamepad1.left_stick_y);
        telemetry.addData("turn", gamepad1.right_stick_x);
        telemetry.update();
    }
}