/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

package frc.lib2960_ctre;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.lib2960.subsystems.*;

/**
 * Motor Mechanism to control the arm shoulder joint
 */
public class MotorMechTalonFX extends MotorMechanismBase {
    
    public final MotorMechTalonFXSettings settings;

    private TalonFX[] motors;                       /**< List of motors */
    private MotorOutputConfigs[] motor_configs;     /**< List of motor output configurations */

    private Encoder quad_encoder;           /**< Quadrature Encoder */
    private DutyCycleEncoder abs_encoder;   /**< Absolute Encoder */

    /**
     * Constructor
     * @param   settings    Settings object
     */
    public MotorMechTalonFX(MotorMechTalonFXSettings settings) {
        super(settings, settings.motor_settings.length);

        this.settings = settings;

        // Initialize Motors
        motors = new TalonFX[settings.motor_settings.length];
        motor_configs = new MotorOutputConfigs[settings.motor_settings.length];

        for(int i = 0; i < motor_count; i++) {
            motors[i] = new TalonFX(settings.motor_settings[i].id);

            // Get current motor configuration
            TalonFXConfigurator configurator = motors[i].getConfigurator();
            motor_configs[i] = new MotorOutputConfigs();
            configurator.refresh(motor_configs[i]);

            // Ovedride Motor Configurations
            motor_configs[i].Inverted = settings.motor_settings[i].inverted ? 
                InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

            // Update motor configuration
            configurator.apply(motor_configs[i]);
        }
        
        // Initialize Encoders
        abs_encoder = new DutyCycleEncoder(settings.abs_enc_settings.port);

        quad_encoder = new Encoder(settings.quad_enc_settings.a_port, 
                                    settings.quad_enc_settings.b_port, 
                                    settings.quad_enc_settings.invert);
        
        quad_encoder.setDistancePerPulse(settings.quad_enc_settings.dpp);

        // Initialize parent class
    }

    /**
     * Sets brake mode on the motors
     * @param   enable  true to enable brake mode. False to disable brake mode.
     */
    public void setBrakeMode(boolean enabled) {
        for(int i = 0; i < motors.length; i++) {
            // Get current motor configuration
            TalonFXConfigurator configurator = motors[i].getConfigurator();
            
            // Ovedride Motor Configurations
            NeutralModeValue mode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            if(mode != motor_configs[i].NeutralMode) {
                motor_configs[i].NeutralMode = mode;

                // Update motor configuration
                configurator.apply(motor_configs[i]);
            }
        }
    }

    /**
     * Get the current position
     * @return  current position
     */
    @Override
    public double getPosition() {
        double current_pos = Rotation2d.fromRotations(abs_encoder.get()).getDegrees();

        // Apply zero offset and invert value if necessary
        if(settings.abs_enc_settings.invert) {
            current_pos = settings.abs_enc_settings.zero_offset - current_pos;
        } else {
            current_pos = current_pos - settings.abs_enc_settings.zero_offset;
        }

        return current_pos;
    }

    /**
     * Get the current rate
     * @return  current rate
     */
    @Override
    public double getRate() {
        return quad_encoder.getRate();
    }

    /**
     * Get the current motor voltage.
     * @param   motor_index     Index of the motor to get the current voltage from.
     * @return  current motor voltage
     */
    @Override
    public double getMotorVoltage(int motor_index) {
        double voltage = 0;

        if(motor_index < motors.length){
            voltage = motors[motor_index].getMotorVoltage().getValueAsDouble();
        } 

        return voltage;
    }

    /**
     * Get the current motor current.
     * @param   motor_index     Index of the motor to get the current current from.
     * @return  current motor current
     */
    @Override
    public double getMotorCurrent(int motor_index) {
        double current = 0;

        if(motor_index < motors.length) {
            current = motors[motor_index].getTorqueCurrent().getValueAsDouble();
        }

        return current;
    }


    /**
     * Sets the motor voltages
     * @param   voltage     motor voltage to set
     */
    @Override
    public void setMotorVoltage(double voltage) {
        for(int i = 0; i < motors.length; i++) motors[i].setVoltage(voltage);
    }
}