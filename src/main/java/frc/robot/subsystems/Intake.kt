package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX
import frc.robot.utils.RobotParameters.IntakeParameters.INTAKE_MOTOR_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.INTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.IntakeParameters.intakeState
import frc.robot.utils.emu.IntakeState

object Intake : SubsystemBase() {
    private val velocitySetter = VelocityTorqueCurrentFOC(0.0)

    private val intakeMotor =
        TonguFX(INTAKE_MOTOR_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = INTAKE_MOTOR_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Intake Motor"
        }

    override fun periodic() {
        setIntakeVelocity(intakeState.velocity)
    }

    fun setIntakeVelocity(speed : Double) {
        intakeMotor.setControl(velocitySetter.withVelocity(speed))
    }
}