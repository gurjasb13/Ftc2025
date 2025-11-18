package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class shootPower extends CommandBase {
    ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

    public shootPower(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        if (gamepad.left_stick_y == 1){
            shooterSubsystem.setPower(-1);
        } else if (gamepad.left_stick_y == -1){
            shooterSubsystem.setPower(1);
        } else{
            shooterSubsystem.setPower(0);
        }
    }

    public void end(){
        shooterSubsystem.stop();
    }
}
