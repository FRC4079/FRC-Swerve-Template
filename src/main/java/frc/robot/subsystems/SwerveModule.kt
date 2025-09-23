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
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import frc.robot.utils.Dash.log
import frc.robot.utils.RobotParameters.MotorParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

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
    private var state: SwerveModuleState
    private var driveVelocity: Double
    private var drivePosition: Double
    private var steerPosition: Double
    private var steerVelocity: Double
    private val driveConfigs: TalonFXConfiguration
    private val steerConfigs: TalonFXConfiguration
    private val driveTorqueConfigs: TorqueCurrentConfigs

    private var driveP: LoggedNetworkNumber? = null
    private var driveI: LoggedNetworkNumber? = null
    private var driveD: LoggedNetworkNumber? = null
    private var driveV: LoggedNetworkNumber? = null

    private var steerP: LoggedNetworkNumber? = null
    private var steerI: LoggedNetworkNumber? = null
    private var steerD: LoggedNetworkNumber? = null
    private var steerV: LoggedNetworkNumber? = null

    private var driveDisconnectedAlert: Alert? = null
    private var turnDisconnectedAlert: Alert? = null
    private var canCoderDisconnectedAlert: Alert? = null

    /**
     * Constructs a new SwerveModule.
     *
     * @param driveId The ID of the drive motor.
     * @param steerId The ID of the steer motor.
     * @param canCoderID The ID of the CANcoder.
     * @param canCoderDriveStraightSteerSetPoint The set point for the CANcoder drive straight steer.
     */
    init {
        state = SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))

        driveConfigs = TalonFXConfiguration()

        // Set the PID values for the drive motor
        driveConfigs.Slot0.kP = PIDParameters.DRIVE_PID_AUTO.getP()
        driveConfigs.Slot0.kI = PIDParameters.DRIVE_PID_AUTO.getI()
        driveConfigs.Slot0.kD = PIDParameters.DRIVE_PID_AUTO.getD()
        driveConfigs.Slot0.kV = PIDParameters.DRIVE_PID_AUTO.v

        // Sets the brake mode, invered, and current limits for the drive motor
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake
        driveConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED
        driveConfigs.CurrentLimits.SupplyCurrentLimit = MotorParameters.DRIVE_SUPPLY_LIMIT
        driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true
        driveConfigs.CurrentLimits.StatorCurrentLimit = MotorParameters.DRIVE_STATOR_LIMIT
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true
        driveConfigs.Feedback.RotorToSensorRatio = MotorParameters.DRIVE_MOTOR_GEAR_RATIO

        steerConfigs = TalonFXConfiguration()

        // Set the PID values for the steer motor
        steerConfigs.Slot0.kP = PIDParameters.STEER_PID_AUTO.getP()
        steerConfigs.Slot0.kI = PIDParameters.STEER_PID_AUTO.getI()
        steerConfigs.Slot0.kD = PIDParameters.STEER_PID_AUTO.getD()
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

        // canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5; TODO: Change
        // default value
        canCoderConfiguration.MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        canCoderConfiguration.MagnetSensor.MagnetOffset =
            SwerveParameters.Thresholds.ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0

        driveMotor.getConfigurator().apply(driveConfigs)
        steerMotor.getConfigurator().apply(steerConfigs)
        canCoder.getConfigurator().apply(canCoderConfiguration)

        driveVelocity = driveMotor.getVelocity().getValueAsDouble()
        drivePosition = driveMotor.getPosition().getValueAsDouble()
        steerVelocity = steerMotor.getVelocity().getValueAsDouble()
        steerPosition = steerMotor.getPosition().getValueAsDouble()

        initializeLoggedNetworkPID()
        intializeAlarms(driveId, steerId, canCoderID)
    }

    val position: SwerveModulePosition
        /**
         * Gets the current position of the swerve module.
         *
         * @return SwerveModulePosition, The current position of the swerve module.
         */
        get() {
            driveVelocity = driveMotor.getVelocity().getValueAsDouble()
            drivePosition = driveMotor.getPosition().getValueAsDouble()
            steerVelocity = steerMotor.getVelocity().getValueAsDouble()
            steerPosition = steerMotor.getPosition().getValueAsDouble()

            swerveModulePosition.angle =
                Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble())
            swerveModulePosition.distanceMeters =
                (drivePosition / MotorParameters.DRIVE_MOTOR_GEAR_RATIO * MotorParameters.METERS_PER_REV)

            return swerveModulePosition
        }

    /**
     * Gets the current state of the swerve module.
     *
     * @return SwerveModuleState, The current state of the swerve module, including the angle and
     * speed.
     */
    fun getState(): SwerveModuleState {
        state.angle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble())
        state.speedMetersPerSecond =
            (
                driveMotor.getRotorVelocity().getValueAsDouble() /
                    MotorParameters.DRIVE_MOTOR_GEAR_RATIO
                    * MotorParameters.METERS_PER_REV
            )
        return state
    }

    /**
     * Sets the state of the swerve module.
     *
     * @param desiredState The desired state of the swerve module.
     */
    fun setState(desiredState: SwerveModuleState) {
        // Get the current angle
        val currentAngle =
            Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble())

        // Optimize the desired state based on current angle
        desiredState.optimize(currentAngle)

        // Set the angle for the steer motor
        val angleToSet = desiredState.angle.getRotations()
        steerMotor.setControl(positionSetter.withPosition(angleToSet))

        // Set the velocity for the drive motor
        val velocityToSet =
            (
                desiredState.speedMetersPerSecond
                    * (MotorParameters.DRIVE_MOTOR_GEAR_RATIO / MotorParameters.METERS_PER_REV)
            )
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet))

        // Log the actual and set values for debugging
        log(
            "drive actual speed " + canCoder.getDeviceID(),
            driveMotor.getVelocity().getValueAsDouble(),
        )
        log("drive set speed " + canCoder.getDeviceID(), velocityToSet)
        log(
            "steer actual angle " + canCoder.getDeviceID(),
            canCoder.getAbsolutePosition().getValueAsDouble(),
        )
        log("steer set angle " + canCoder.getDeviceID(), angleToSet)
        log(
            "desired state after optimize " + canCoder.getDeviceID(),
            desiredState.angle.getRotations(),
        )

        // Update the state with the optimized values
        state = desiredState
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
        pid: PIDController,
        velocity: Double,
    ) {
        driveConfigs.Slot0.kP = pid.getP()
        driveConfigs.Slot0.kI = pid.getI()
        driveConfigs.Slot0.kD = pid.getD()
        driveConfigs.Slot0.kV = velocity
        driveMotor.getConfigurator().apply(driveConfigs)
    }

    /**
     * Sets the steer PID values.
     *
     * @param pid The PID object containing the PID values.
     * @param velocity The velocity value.
     */
    fun setSteerPID(
        pid: PIDController,
        velocity: Double,
    ) {
        steerConfigs.Slot0.kP = pid.getP()
        steerConfigs.Slot0.kI = pid.getI()
        steerConfigs.Slot0.kD = pid.getD()
        steerConfigs.Slot0.kV = velocity
        steerMotor.getConfigurator().apply(steerConfigs)
    }

    fun applyTelePIDValues() {
        driveConfigs.Slot0.kP = driveP!!.get()
        driveConfigs.Slot0.kI = driveI!!.get()
        driveConfigs.Slot0.kD = driveD!!.get()
        driveConfigs.Slot0.kV = driveV!!.get()

        steerConfigs.Slot0.kP = steerP!!.get()
        steerConfigs.Slot0.kI = steerI!!.get()
        steerConfigs.Slot0.kD = steerD!!.get()
        steerConfigs.Slot0.kV = steerV!!.get()

        driveMotor.getConfigurator().apply(driveConfigs)
        steerMotor.getConfigurator().apply(steerConfigs)
    }

    /** Sets the PID values for teleoperation mode.  */
    fun setTelePID() {
        setDrivePID(PIDParameters.DRIVE_PID_TELE, PIDParameters.DRIVE_PID_TELE.v)
        setSteerPID(PIDParameters.STEER_PID_TELE, PIDParameters.STEER_PID_TELE.v)
    }

    /** Sets the PID values for autonomous mode.  */
    fun setAutoPID() {
        setDrivePID(PIDParameters.DRIVE_PID_AUTO, PIDParameters.DRIVE_PID_AUTO.v)
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

    fun intializeAlarms(
        driveID: Int,
        steerID: Int,
        canCoderID: Int,
    ) {
        driveDisconnectedAlert =
            Alert("Disconnected drive motor " + driveID.toString() + ".", AlertType.kError)
        turnDisconnectedAlert =
            Alert("Disconnected turn motor " + steerID.toString() + ".", AlertType.kError)
        canCoderDisconnectedAlert =
            Alert("Disconnected CANCoder " + canCoderID.toString() + ".", AlertType.kError)

        driveDisconnectedAlert!!.set(!driveMotor.isConnected())
        turnDisconnectedAlert!!.set(!steerMotor.isConnected())
        canCoderDisconnectedAlert!!.set(!canCoder.isConnected())
    }

    fun updateTelePID() {
        PIDParameters.DRIVE_PID_TELE.setP(driveP!!.get())
        PIDParameters.DRIVE_PID_TELE.setI(driveI!!.get())
        PIDParameters.DRIVE_PID_TELE.setD(driveD!!.get())

        PIDParameters.STEER_PID_TELE.setP(steerP!!.get())
        PIDParameters.STEER_PID_TELE.setI(steerI!!.get())
        PIDParameters.STEER_PID_TELE.setD(steerD!!.get())

        applyTelePIDValues()
    }
}
