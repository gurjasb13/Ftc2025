package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ShooterSubsystem extends SubsystemBase {
    private final DcMotorEx shooterMotor1;

    private static final double TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 4/3;
    private static final double TICKS_PER_REV_OUTPUT = TICKS_PER_REV * GEAR_RATIO;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");

        shooterMotor1.setDirection(DcMotor.Direction.FORWARD);

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getCurrentRPM() {
        double ticksPerSec = shooterMotor1.getVelocity();
        double currentRPM = (ticksPerSec * 60.0) / TICKS_PER_REV_OUTPUT;
        return currentRPM;
    }

    public void setPower(double power) {
        shooterMotor1.setPower(power);
    }

    public void stop() {
        shooterMotor1.setPower(0);
    }
}