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

import frc.lib2960.controllers.PositionControllerSettings;
import frc.lib2960.subsystems.MotorMechStageSettings;
import frc.lib2960.subsystems.MotorMechanismBaseSettings;
import frc.lib2960.util.*;

/**
 * Motor mechanism settings for a TalonFX motor mechanism
 */
public class MotorMechTalonFXSettings  extends MotorMechanismBaseSettings {
    public final MotorSettings[] motor_settings;            /**< List of Motor settings */
    public final QuadEncoderSettings quad_enc_settings;     /**< Quadurature Encoder Settings */
    public final AbsEncoderSettings abs_enc_settings;       /**< Absolute Encoder Settings */
    
    /**
     * Constructor. All values should be in degrees.
     * @param   name                Name of the mechanism
     * @param   tab_name            ShuffleBoard tab name
     * @param   pos_ctrl            Position Controller settings
     * @param   stage_settings      List of state settings
     * @param   def_tol             Default position tolerances
     * @param   motor_settings      List of Motor settings
     * @param   quad_enc_settings   Quadurature Encoder Settings
     * @param   abs_enc_settings    Absolute Encoder Settings
     */
    public MotorMechTalonFXSettings(String name, String tab_name, PositionControllerSettings pos_ctrl, 
                MotorMechStageSettings[] stage_settings, Limits def_tol,
                MotorSettings[] motor_settings,
                QuadEncoderSettings quad_enc_settings,
                AbsEncoderSettings abs_enc_settings) {
        
        super(name, tab_name, pos_ctrl, stage_settings, def_tol);

        this.motor_settings = motor_settings;
        this.quad_enc_settings = quad_enc_settings;
        this.abs_enc_settings = abs_enc_settings;
    }
}