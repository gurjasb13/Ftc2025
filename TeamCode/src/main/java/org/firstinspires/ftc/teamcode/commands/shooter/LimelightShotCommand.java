package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class LimelightShotCommand extends CommandBase {
    private ShooterSubsystem shooter;
    private Limelight3A limelight;
    private final double cameraHeight = 5;
    private final double targetHeight = 29;
    private final double cameraAngle = Math.toRadians(70);

    public LimelightShotCommand(ShooterSubsystem shooter, Limelight3A limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(shooter);
    }

    public double getDistance(double ty) {
        double angle = cameraAngle + Math.toRadians(ty);
        return (targetHeight - cameraHeight) / Math.tan(angle);
    }

    @Override
    public void execute() {
        LLResult r = limelight.getLatestResult();
        if (r != null) {
            double dist = getDistance(r.getTy());
            shooter.runLimelightShot(dist);
        }
    }
}