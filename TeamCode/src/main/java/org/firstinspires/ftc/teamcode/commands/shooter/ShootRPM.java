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

    public static double kP = 0.0005;
    public static double kD = 0.0001;

    private double targetRPM = 0;

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

        double power = controller.calculate(shooterSubsystem.getCurrentRPM(), targetRPM);
        power = Math.max(0, Math.min(power, 1));

        if (gamepad.a == true){
            shooterSubsystem.setPower(power);
        }
    }

    public void end(){
        shooterSubsystem.stop();
    }

    public boolean isFinished(){
        return true;
    }
}
