package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class shootPower extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;

    public shootPower(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute(){
        double power = gamepad.a ? 1 : 0;
        shooterSubsystem.setPower(power);
    }

    public void end(){
        shooterSubsystem.stop();
    }

    public boolean isFinished(){
        return true;
    }
}
