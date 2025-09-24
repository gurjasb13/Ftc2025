package org.firstinspires.ftc.teamcode.commands.drivebase;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;

public class RobotCentricCommand extends CommandBase{
    private final DrivebaseSubsystem drivebaseSubsystem;
    private final Gamepad gamepad;

    public RobotCentricCommand(DrivebaseSubsystem drivebaseSubsystem, Gamepad gamepad){
        this.drivebaseSubsystem = drivebaseSubsystem;
        this.gamepad = gamepad;
        addRequirements(drivebaseSubsystem);
    }

    public void execute(){
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        drivebaseSubsystem.drive(x, y, rx);
    }
}