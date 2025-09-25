package frc.robot.subsystems

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.configs.TorqueCurrentConfigs
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.robot.utils.RobotParameters.MotorParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.Pingu

/** Represents a swerve module used in a swerve drive system.  */
class SwerveModule(
    driveId: Int,
    steerId: Int,
    canCoderID: Int,
    canCoderDriveStraightSteerSetPoint: Double,
) {
    private val driveMotor: TalonFX = TalonFX(driveId)
    private val canCoder: CANcoder = CANcoder(canCoderID)
    private val steerMotor: TalonFX = TalonFX(steerId)
    private val positionSetter: PositionTorqueCurrentFOC = PositionTorqueCurrentFOC(0.0)
    private val velocitySetter: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    private val swerveModulePosition: SwerveModulePosition = SwerveModulePosition()
    private var driveVelocity: Double
    private var drivePosition: Double
    private var steerPosition: Double
    private var steerVelocity: Double
    private val driveConfigs: TalonFXConfiguration = TalonFXConfiguration()
    private val steerConfigs: TalonFXConfiguration
    private val driveTorqueConfigs: TorqueCurrentConfigs

    private lateinit var driveP: LoggedNetworkNumber
    private lateinit var driveI: LoggedNetworkNumber
    private lateinit var driveD: LoggedNetworkNumber
    private lateinit var driveV: LoggedNetworkNumber

    private lateinit var steerP: LoggedNetworkNumber
    private lateinit var steerI: LoggedNetworkNumber
    private lateinit var steerD: LoggedNetworkNumber
    private lateinit var steerV: LoggedNetworkNumber

    private lateinit var driveDisconnectedAlert: Alert
    private lateinit var turnDisconnectedAlert: Alert
    private lateinit var canCoderDisconnectedAlert: Alert

    /**
     * Gets the current position of the swerve module.
     *
     * Updates the cached drive and steer velocities and positions,
     * then sets the swerve module's angle and distance in meters
     * based on the current sensor readings.
     *
     * @return The current `SwerveModulePosition` containing angle and distance.
     */
    val position: SwerveModulePosition
        get() {
            driveVelocity = driveMotor.velocity.valueAsDouble
            drivePosition = driveMotor.position.valueAsDouble
            steerVelocity = steerMotor.velocity.valueAsDouble
            steerPosition = steerMotor.position.valueAsDouble

            swerveModulePosition.angle =
                Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)
            swerveModulePosition.distanceMeters =
                (drivePosition / MotorParameters.DRIVE_MOTOR_GEAR_RATIO * MotorParameters.METERS_PER_REV)

            return swerveModulePosition
        }

    var state = SwerveModuleState()
        get() {
            field.angle = Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)
            field.speedMetersPerSecond =
                (
                    driveMotor.rotorVelocity.valueAsDouble /
                        MotorParameters.DRIVE_MOTOR_GEAR_RATIO
                        * MotorParameters.METERS_PER_REV
                )
            return field
        }
        set(value) {
            // Get the current angle
            val currentAngle =
                Rotation2d.fromRotations(canCoder.absolutePosition.valueAsDouble)

            // Optimize the desired state based on current angle
            value.optimize(currentAngle)

            // Set the angle for the steer motor
            val angleToSet = value.angle.rotations
            steerMotor.setControl(positionSetter.withPosition(angleToSet))

            // Set the velocity for the drive motor
            val velocityToSet =
                (
                    value.speedMetersPerSecond
                        * (MotorParameters.DRIVE_MOTOR_GEAR_RATIO / MotorParameters.METERS_PER_REV)
                )
            driveMotor.setControl(velocitySetter.withVelocity(velocityToSet))

            // Log the actual and set values for debugging
            logs {
                log("drive actual speed " + canCoder.deviceID, driveMotor.velocity.valueAsDouble)
                log("drive set speed " + canCoder.deviceID, velocityToSet)
                log("steer actual angle " + canCoder.deviceID, canCoder.absolutePosition.valueAsDouble)
                log("steer set angle " + canCoder.deviceID, angleToSet)
                log("desired state after optimize " + canCoder.deviceID, value.angle.rotations)
            }

            // Update the state with the optimized values
            field = value
        }

    /**
     * Constructs a new SwerveModule.
     *
     * @param driveId The ID of the drive motor.
     * @param steerId The ID of the steer motor.
     * @param canCoderID The ID of the CANcoder.
     * @param canCoderDriveStraightSteerSetPoint The set point for the CANcoder drive straight steer.
     */
    init {

        // Set the PID values for the drive motor
        driveConfigs.Slot0.kP = PIDParameters.DRIVE_PID_AUTO.p
        driveConfigs.Slot0.kI = PIDParameters.DRIVE_PID_AUTO.i
        driveConfigs.Slot0.kD = PIDParameters.DRIVE_PID_AUTO.d
        driveConfigs.Slot0.kV = PIDParameters.DRIVE_PID_AUTO.v!!

        // Sets the brake mode, inverted, and current limits for the drive motor
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        driveConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED
        driveConfigs.CurrentLimits.SupplyCurrentLimit = MotorParameters.DRIVE_SUPPLY_LIMIT
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true
        driveConfigs.CurrentLimits.StatorCurrentLimit = MotorParameters.DRIVE_STATOR_LIMIT
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfigs.Feedback.RotorToSensorRatio = MotorParameters.DRIVE_MOTOR_GEAR_RATIO

        steerConfigs = TalonFXConfiguration()

        // Set the PID values for the steer motor
        steerConfigs.Slot0.kP = PIDParameters.STEER_PID_AUTO.p
        steerConfigs.Slot0.kI = PIDParameters.STEER_PID_AUTO.i
        steerConfigs.Slot0.kD = PIDParameters.STEER_PID_AUTO.d
        steerConfigs.Slot0.kV = 0.0
        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true

        // Sets the brake mode, inverted, and current limits for the steer motor
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        steerConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.STEER_MOTOR_INVERTED
        steerConfigs.Feedback.FeedbackRemoteSensorID = canCoderID
        steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
        steerConfigs.Feedback.RotorToSensorRatio = MotorParameters.STEER_MOTOR_GEAR_RATIO
        steerConfigs.CurrentLimits.SupplyCurrentLimit = MotorParameters.STEER_SUPPLY_LIMIT
        steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true

        driveTorqueConfigs = TorqueCurrentConfigs()

        val canCoderConfiguration = CANcoderConfiguration()

        // Sets the CANCoder direction, absolute sensor range, and magnet offset for the CANCoder Make
        // sure the magnet offset is ACCURATE and based on when the wheel is straight!

        // canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        // TODO: Change default value
        canCoderConfiguration.MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        canCoderConfiguration.MagnetSensor.MagnetOffset =
            SwerveParameters.Thresholds.ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0

        driveMotor.configurator.apply(driveConfigs)
        steerMotor.configurator.apply(steerConfigs)
        canCoder.configurator.apply(canCoderConfiguration)

        driveVelocity = driveMotor.velocity.valueAsDouble
        drivePosition = driveMotor.position.valueAsDouble
        steerVelocity = steerMotor.velocity.valueAsDouble
        steerPosition = steerMotor.position.valueAsDouble

        initializeLoggedNetworkPID()
        initializeAlarms(driveId, steerId, canCoderID)
    }

    /** Stops the swerve module motors.  */
    fun stop() {
        steerMotor.stopMotor()
        driveMotor.stopMotor()
    }

    /**
     * Sets the drive PID values.
     *
     * @param pid The PID object containing the PID values.
     * @param velocity The velocity value.
     */
    fun setDrivePID(
        pid: Pingu,
        velocity: Double,
    ) {
        driveConfigs.Slot0.kP = pid.p
        driveConfigs.Slot0.kI = pid.i
        driveConfigs.Slot0.kD = pid.d
        driveConfigs.Slot0.kV = velocity
        driveMotor.configurator.apply(driveConfigs)
    }

    /**
     * Sets the steer PID values.
     *
     * @param pid The PID object containing the PID values.
     * @param velocity The velocity value.
     */
    fun setSteerPID(
        pid: Pingu,
        velocity: Double,
    ) {
        steerConfigs.Slot0.kP = pid.p
        steerConfigs.Slot0.kI = pid.i
        steerConfigs.Slot0.kD = pid.d
        steerConfigs.Slot0.kV = velocity
        steerMotor.configurator.apply(steerConfigs)
    }

    fun applyTelePIDValues() {
        driveConfigs.Slot0.kP = driveP.get()
        driveConfigs.Slot0.kI = driveI.get()
        driveConfigs.Slot0.kD = driveD.get()
        driveConfigs.Slot0.kV = driveV.get()

        steerConfigs.Slot0.kP = steerP.get()
        steerConfigs.Slot0.kI = steerI.get()
        steerConfigs.Slot0.kD = steerD.get()
        steerConfigs.Slot0.kV = steerV.get()

        driveMotor.configurator.apply(driveConfigs)
        steerMotor.configurator.apply(steerConfigs)
    }

    /** Sets the PID values for teleoperation mode.  */
    fun setTelePID() {
        setDrivePID(PIDParameters.DRIVE_PID_TELE, PIDParameters.DRIVE_PID_TELE.v!!)
        setSteerPID(PIDParameters.STEER_PID_TELE, PIDParameters.STEER_PID_TELE.v!!)
    }

    /** Sets the PID values for autonomous mode.  */
    fun setAutoPID() {
        setDrivePID(PIDParameters.DRIVE_PID_AUTO, PIDParameters.DRIVE_PID_AUTO.v!!)
    }

    /** Resets the drive motor position to zero.  */
    fun resetDrivePosition() {
        driveMotor.setPosition(0.0)
    }

    fun initializeLoggedNetworkPID() {
        driveP = LoggedNetworkNumber("/Tuning/Drive P", driveConfigs.Slot0.kP)
        driveI = LoggedNetworkNumber("/Tuning/Drive I", driveConfigs.Slot0.kI)
        driveD = LoggedNetworkNumber("/Tuning/Drive D", driveConfigs.Slot0.kD)
        driveV = LoggedNetworkNumber("/Tuning/Drive V", driveConfigs.Slot0.kV)

        steerP = LoggedNetworkNumber("/Tuning/Steer P", steerConfigs.Slot0.kP)
        steerI = LoggedNetworkNumber("/Tuning/Steer I", steerConfigs.Slot0.kI)
        steerD = LoggedNetworkNumber("/Tuning/Steer D", steerConfigs.Slot0.kD)
        steerV = LoggedNetworkNumber("/Tuning/Steer V", steerConfigs.Slot0.kV)
    }

    fun initializeAlarms(
        driveID: Int,
        steerID: Int,
        canCoderID: Int,
    ) {
        driveDisconnectedAlert =
            Alert("Disconnected drive motor $driveID.", AlertType.kError)
        turnDisconnectedAlert =
            Alert("Disconnected turn motor $steerID.", AlertType.kError)
        canCoderDisconnectedAlert =
            Alert("Disconnected CANCoder $canCoderID.", AlertType.kError)

        driveDisconnectedAlert.set(!driveMotor.isConnected)
        turnDisconnectedAlert.set(!steerMotor.isConnected)
        canCoderDisconnectedAlert.set(!canCoder.isConnected)
    }

    fun updateTelePID() {
        PIDParameters.DRIVE_PID_TELE.p = driveP.get()
        PIDParameters.DRIVE_PID_TELE.i = driveI.get()
        PIDParameters.DRIVE_PID_TELE.d = driveD.get()

        PIDParameters.STEER_PID_TELE.p = steerP.get()
        PIDParameters.STEER_PID_TELE.i = steerI.get()
        PIDParameters.STEER_PID_TELE.d = steerD.get()

        applyTelePIDValues()
    }
}
