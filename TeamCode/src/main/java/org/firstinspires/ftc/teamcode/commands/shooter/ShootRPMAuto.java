package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PDController;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRPMAuto extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;
    private final PDController controller;

    private final double targetRPM;
    private static final double RPM_TOLERANCE = 75; // change if needed

    public static double kP = 0.25;
    public static double kD = 0.01;

    public ShootRPMAuto(ShooterSubsystem shooterSubsystem, double targetRPM) {
        this.shooterSubsystem = shooterSubsystem;
        this.targetRPM = targetRPM;

        addRequirements(shooterSubsystem);

        controller = new PDController(kP, kD);
    }

    @Override
    public void execute() {
        controller.setP(kP);
        controller.setD(kD);

        double power = controller.calculate(
                shooterSubsystem.getCurrentRPM(),
                targetRPM
        );

        power = Math.max(0, Math.min(power, 1));

        shooterSubsystem.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(shooterSubsystem.getCurrentRPM() - targetRPM) < RPM_TOLERANCE;
    }
}
