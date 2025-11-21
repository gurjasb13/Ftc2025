package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {

    private final DcMotorEx shooterMotor1;

    // motor specs
    private static final double TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 4.0 / 3.0;   // your value, but now correct double
    private static final double TICKS_PER_REV_OUTPUT = TICKS_PER_REV * GEAR_RATIO;

    // controller
    private final PDController controller;

    // tunables
    public static double kP = 0.01;
    public static double kD = 0.01;
    public static double kF = 0.20;   // baseline feedforward to overcome friction

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");

        shooterMotor1.setDirection(DcMotor.Direction.REVERSE);

        shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PDController(kP, kD);
    }

    // ---- Utility ----

    public double getCurrentRPM() {
        double ticksPerSec = shooterMotor1.getVelocity();
        return (ticksPerSec * 60.0) / TICKS_PER_REV_OUTPUT;
    }

    public void setPower(double power) {
        shooterMotor1.setPower(power);
    }

    public void stop() {
        shooterMotor1.setPower(0);
    }

    // ---- Actual RPM Control ----

    public void runToRPM(double targetRPM) {

        double currentRPM = getCurrentRPM();

        controller.setP(kP);
        controller.setD(kD);

        // PD correction (note the FIXED order)
        double correction = controller.calculate(targetRPM, currentRPM);

        // baseline power to overcome friction + spin up
        double ff = kF * (targetRPM / 6000.0);
        // scale FF to target RPM (customize if needed)

        double power = ff + correction;

        // clamp properly: allow negative to slow down
        power = Math.max(-1, Math.min(1.0, power));

        shooterMotor1.setPower(-power);
    }
}
