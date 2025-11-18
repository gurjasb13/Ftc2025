package org.firstinspires.ftc.teamcode.commands.intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeAction implements Action {
    private final IntakeSubsystem intake;
    private final double power;
    private final double duration;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeAction(IntakeSubsystem intake, double power, double duration) {
        this.intake = intake;
        this.power = power;
        this.duration = duration;
    }

    @Override
    public boolean run(TelemetryPacket packet) {
        // Start intake
        intake.setPower(power);

        // Stop after duration
        if (timer.seconds() >= duration) {
            intake.stop();
            return false; // finished
        }

        return true; // keep going
    }
}
