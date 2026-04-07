package org.firstinspires.ftc.teamcode.commands.shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.DrivebaseSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRPM extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final Gamepad gamepad;
    private final PDController controller;

    public static double kP = 0.6;
    public static double kD = 0.07;


    public ShootRPM(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
        addRequirements(shooterSubsystem);

        controller = new PDController(kP, kD);
    }

    @Override
    public void execute(){
        if(gamepad.a) {
            shooterSubsystem.runToRPM(4500);
            if(Math.abs(shooterSubsystem.getCurrentRPM() - 4300) <= 200){
                gamepad.rumble(2000);
            }
        } else if (gamepad.b) {
            shooterSubsystem.runToRPM(5600);
            if(Math.abs(shooterSubsystem.getCurrentRPM() - 5500) <= 300){
                gamepad.rumble(2000);
            }
        } else if(gamepad.x){
            shooterSubsystem.setPower(-1);
        }
        else{shooterSubsystem.setPower(0);
        }
    }

    public void end(){
        shooterSubsystem.stop();
    }

    public boolean isFinished(){
        return true;
    }
}
