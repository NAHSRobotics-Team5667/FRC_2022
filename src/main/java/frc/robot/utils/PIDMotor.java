package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class PIDMotor implements MotorController {
    private WPI_TalonFX motor;
    private PIDController speedController;

    public PIDMotor(WPI_TalonFX motor, double kp, double ki, double kd) {
        this.motor = motor;
        speedController = new PIDController(kp, ki, kd);
    }

    public void set(double input) {
        if (input == 0) {
            motor.set(speedController.calculate(motor.get(), 0));
        } else {
            motor.set(input);
        }
    }

    @Override
    public double get() {
        return motor.get();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(true);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.set(0);
        
    }

    @Override
    public void stopMotor() {
        motor.set(0);
    }
}
