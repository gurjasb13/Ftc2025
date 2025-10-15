package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRPM extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetRPM;

    public ShootRPM(ShooterSubsystem shooterSubsystem, double targetRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM=targetRPM;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTargetRPM(targetRPM);
    }

    public void execute(){
        shooterSubsystem.setTargetRPM(targetRPM);
    }

    public void end(){
        shooterSubsystem.stop();
    }

    public boolean isFinished(){
        return true;
    }
}
