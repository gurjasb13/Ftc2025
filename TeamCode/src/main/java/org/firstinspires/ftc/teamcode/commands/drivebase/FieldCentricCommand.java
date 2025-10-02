package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class FieldCentricCommand extends CommandBase {
    DrivebaseSubsystem drivebaseSubsystem;
    Gamepad gamepad;

    public FieldCentricCommand(DrivebaseSubsystem drivebaseSubsystem, Gamepad gamepad){
        drivebaseSubsystem=this.drivebaseSubsystem;
        gamepad=this.gamepad;
    }

    @Override
    public void execute() {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        if (gamepad.options){
            drivebaseSubsystem.resetIMU();
        }

        drivebaseSubsystem.drive(x, y, rx);
    }
}
