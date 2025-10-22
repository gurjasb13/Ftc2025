package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooterMotor1;
    private final DcMotorEx shooterMotor2;
    private final PDController controller;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double kP = 0.0005;
    public static double kD = 0.0001;
    private static final double TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 4/3;
    private static final double TICKS_PER_REV_OUTPUT = TICKS_PER_REV * GEAR_RATIO;

    private double targetRPM = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");

        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor2.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PDController(kP, kD);
    }

    public void setTargetRPM(double targetRPM) {

        this.targetRPM = targetRPM;
    }

    public void setPower(double power) {
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);
    }

    public void stop() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    /*@Override
    public void periodic() {
        controller.setP(kP);
        controller.setD(kD);

        double ticksPerSec = shooterMotor1.getVelocity();
        double currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV_OUTPUT;

        double power = controller.calculate(currentRPM, targetRPM);
        power = Math.max(0, Math.min(power, 1));

        shooterMotor1.setPower(power);
        shooterMotor2.setPower(power);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRPM);
        packet.put("Current RPM", currentRPM);
        packet.put("Power", power);
        dashboard.sendTelemetryPacket(packet);
    }*/
}