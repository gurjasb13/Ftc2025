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

    public static double kP = 0.25;
    public static double kD = 0.01;


    public ShootRPM(ShooterSubsystem shooterSubsystem, Gamepad gamepad) {
        this.shooterSubsystem = shooterSubsystem;
        this.gamepad = gamepad;
        addRequirements(shooterSubsystem);

        controller = new PDController(kP, kD);
    }

    @Override
    public void execute(){
        controller.setP(kP);
        controller.setD(kD);

        double closePower = controller.calculate(shooterSubsystem.getCurrentRPM(), 3000);
        closePower = Math.max(0, Math.min(closePower, 1));

        double mediumPower = controller.calculate(shooterSubsystem.getCurrentRPM(), 3700);
        closePower = Math.max(0, Math.min(mediumPower, 1));

        double farPower = controller.calculate(shooterSubsystem.getCurrentRPM(), 5700);
        farPower = Math.max(0, Math.min(farPower, 1));

        if (gamepad.x){
            shooterSubsystem.setPower(closePower);
        }else if(gamepad.a) {
            shooterSubsystem.setPower(mediumPower);
        } else if (gamepad.b) {
            shooterSubsystem.setPower(farPower);
        }
        else{
            shooterSubsystem.setPower(0);
        }
    }

    public void end(){
        shooterSubsystem.stop();
    }

    public boolean isFinished(){
        return true;
    }
}
