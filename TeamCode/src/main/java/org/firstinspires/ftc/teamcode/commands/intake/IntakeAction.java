package org.firstinspires.ftc.teamcode.commands.intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeAction implements Action {
    private final IntakeSubsystem intake;
    private final double power;
    private boolean done = false;

    public IntakeAction(IntakeSubsystem intake, double power) {
        this.intake = intake;
        this.power = power;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        intake.setPower(power);
        // You could add timing or logic to stop after X seconds
        return !done;
    }
}
