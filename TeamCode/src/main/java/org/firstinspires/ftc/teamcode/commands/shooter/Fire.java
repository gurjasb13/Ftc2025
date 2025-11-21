package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gate;

public class Fire extends CommandBase {

    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;
    private final Gate gate;
    private final int rpm;

    public Fire(ShooterSubsystem shooter, IntakeSubsystem intake, Gate gate, int rpm) {
        this.shooter = shooter;
        this.intake = intake;
        this.gate = gate;
        this.rpm = rpm;

        addRequirements(shooter, intake, gate);
    }

    @Override
    public void execute() {
        shooter.runToRPM(rpm);
        intake.setPower(1);
        gate.setPosition(0);
    }

    @Override
    public boolean isFinished() {
        return true; // instant fire
    }
}