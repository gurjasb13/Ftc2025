package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Config
@TeleOp(name = "Shooter PID Dashboard Tuning", group = "Tuning")
public class ShooterPIDDashboardTuning extends OpMode {

    // 🔥 DASHBOARD TUNABLES
    public static double TARGET_RPM = 3000;
    public static double kP = 0.02;
    public static double kD = 0.01;
    public static double kF = 0.20;

    private ShooterSubsystem shooter;

    @Override
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        // push dashboard values into subsystem
        ShooterSubsystem.kP = kP;
        ShooterSubsystem.kD = kD;
        ShooterSubsystem.kF = kF;

        shooter.runToRPM(TARGET_RPM);

        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", shooter.getCurrentRPM());
        telemetry.addData("kP", kP);
        telemetry.addData("kD", kD);
        telemetry.addData("kF", kF);
        telemetry.update();
    }
}
