package frc.robot.subsystems

import com.ctre.phoenix6.hardware.Pigeon2
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.MotorParameters
import frc.robot.utils.RobotParameters.MotorParameters.MAX_ANGULAR_SPEED
import frc.robot.utils.RobotParameters.MotorParameters.MAX_SPEED
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters
import frc.robot.utils.RobotParameters.SwerveParameters.swerveState
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.SHOULD_INVERT
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE
import frc.robot.utils.RobotParameters.SwerveParameters.slowmode
import frc.robot.utils.RobotParameters.GameParameters.teamColor
import org.photonvision.EstimatedRobotPose
import xyz.malefic.frc.pingu.control.Pingu
import xyz.malefic.frc.pingu.log.LogPingu.log
import java.util.function.BooleanSupplier
import frc.robot.utils.emu.SwerveDriveState
import kotlin.math.abs


object Swerve : SubsystemBase() {
    private val poseEstimator: SwerveDrivePoseEstimator
    private val field = Field2d()
    private val pidgey = Pigeon2(MotorParameters.PIDGEY_ID)
    private val states = arrayOfNulls<SwerveModuleState>(4)
    private var setStates = arrayOfNulls<SwerveModuleState>(4)
    private val modules: Array<SwerveModule>
    private val pid: Pingu
    private var currentPose: Pose2d? = Pose2d(0.0, 0.0, Rotation2d(0.0))
    private var pathToScore: PathPlannerPath? = null


    var swerveLoggingThread: Thread =
        Thread {
            while (true) {
                "Swerve Module States" log this.moduleStates
                try {
                    Thread.sleep(100)
                } catch (e: InterruptedException) {
                    e.printStackTrace()
                    Thread.currentThread().interrupt()
                    break
                }
            }
        }

    var swerveLoggingThreadBeforeSet: Thread =
        Thread {
            while (true) {
                "Set Swerve Module States" log setStates
                try {
                    Thread.sleep(100)
                } catch (e: InterruptedException) {
                    e.printStackTrace()
                    Thread.currentThread().interrupt()
                    break
                }
            }
        }

    // from feeder to the goal and align itself
    // The plan is for it to path towards it then we use a set path to align itself with the goal and
    // be more accurate
    // Use this https://pathplanner.dev/pplib-pathfinding.html#pathfind-then-follow-path
    var constraints: PathConstraints =
        PathConstraints(2.0, 3.0, Units.degreesToRadians(540.0), Units.degreesToRadians(720.0))

    init {
        this.modules = initializeModules()
        this.pid = initializePID()
        this.pidgey.reset()
        this.poseEstimator = initializePoseEstimator()
        configureAutoBuilder()
        swerveLoggingThread.start()
        swerveLoggingThreadBeforeSet.start()

        try {
            pathToScore = PathPlannerPath.fromPathFile("Straight Path")
        } catch (e: Exception) {
            throw RuntimeException("Failed to load robot config", e)
        }
    }

    /**
     * Initializes the swerve modules. Ensure the swerve modules are initialized in the same order as
     * in kinematics.
     *
     * @return SwerveModule[], An array of initialized SwerveModule objects.
     */
    private fun initializeModules(): Array<SwerveModule> =
        arrayOf<SwerveModule>(
            SwerveModule(
                MotorParameters.FRONT_LEFT_DRIVE_ID,
                MotorParameters.FRONT_LEFT_STEER_ID,
                MotorParameters.FRONT_LEFT_CAN_CODER_ID,
                SwerveParameters.Thresholds.CANCODER_VAL9,
            ),
            SwerveModule(
                MotorParameters.FRONT_RIGHT_DRIVE_ID,
                MotorParameters.FRONT_RIGHT_STEER_ID,
                MotorParameters.FRONT_RIGHT_CAN_CODER_ID,
                SwerveParameters.Thresholds.CANCODER_VAL10,
            ),
            SwerveModule(
                MotorParameters.BACK_LEFT_DRIVE_ID,
                MotorParameters.BACK_LEFT_STEER_ID,
                MotorParameters.BACK_LEFT_CAN_CODER_ID,
                SwerveParameters.Thresholds.CANCODER_VAL11,
            ),
            SwerveModule(
                MotorParameters.BACK_RIGHT_DRIVE_ID,
                MotorParameters.BACK_RIGHT_STEER_ID,
                MotorParameters.BACK_RIGHT_CAN_CODER_ID,
                SwerveParameters.Thresholds.CANCODER_VAL12,
            ),
        )

    /**
     * Initializes the PID controller.
     *
     * @return PIDController, A new PID object with values from the SmartDashboard.
     */
    private fun initializePID(): Pingu =
        Pingu(
            SmartDashboard.getNumber("AUTO: P", PIDParameters.DRIVE_PID_AUTO.p),
            SmartDashboard.getNumber("AUTO: I", PIDParameters.DRIVE_PID_AUTO.i),
            SmartDashboard.getNumber("AUTO: D", PIDParameters.DRIVE_PID_AUTO.d),
            SmartDashboard.getNumber("AUTO: V", PIDParameters.DRIVE_PID_AUTO.v),
        )

    /**
     * Initializes the SwerveDrivePoseEstimator. The SwerveDrivePoseEsimator estimates the robot's
     * position. This is based on a combination of the robot's movement and vision.
     *
     * @return SwerveDrivePoseEstimator, A new SwerveDrivePoseEstimator object.
     */
    private fun initializePoseEstimator(): SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(
            SwerveParameters.PhysicalParameters.kinematics,
            Rotation2d.fromDegrees(this.heading),
            this.modulePositions,
            Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        )

    /**
     * Configures the AutoBuilder for autonomous driving. READ DOCUMENTATION TO PUT IN CORRECT VALUES
     * Allows PathPlanner to get pose and output robot-relative chassis speeds Needs tuning
     */
    private fun configureAutoBuilder() {
        checkNotNull(PIDParameters.config)
        AutoBuilder.configure(
            { this.pose },
            { pose: Pose2d? -> this.newPose(pose) },
            { this.autoSpeeds },
            { chassisSpeeds: ChassisSpeeds? -> this.chassisSpeedsDrive(chassisSpeeds) },
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            PIDParameters.config,
            BooleanSupplier {
                val alliance = DriverStation.getAlliance()
                if (alliance.isEmpty) {
                    return@BooleanSupplier false
                }
                return@BooleanSupplier if (SHOULD_INVERT) {
                    alliance.get() == Alliance.Red
                } else {
                    alliance.get() != Alliance.Blue
                }
            },
            this,
        )
    }

    /**
     * Applies the current swerve drive state to the swerve modules. Should be called in [periodic].
     *
     * - When in [SwerveDriveState.FIELD_ORIENTED], moves the robot according to the driver input using the wrapper function [fieldDriveUsingController].
     * - When in [SwerveDriveState.SHOOTING], restricts driver input to only movement, no rotation...as long as we are using a non-turret. Two auto-heading behaviors
     *      - If in shooting zone, Robot heading automatically points towards team hub
     *      - If in the center, Robot heading offsets to the closest side of the team hub
     */

    private fun applySwerveState() {
        when (swerveState) {
            SwerveDriveState.FIELD_ORIENTED -> {
                fieldDriveUsingController()
            }
            SwerveDriveState.SHOOTING -> {
            }
        }
    }

    /**
     * This method is called periodically by the scheduler. It updates the pose estimator and
     * dashboard values.
     */
    override fun periodic() {
        /*
             This method checks whether the bot is in Teleop, and adds it to poseEstimator based on VISION
         */

        if (DriverStation.isTeleop()) {
            val estimatedPose: EstimatedRobotPose? =
                PhotonVision.getEstimatedGlobalPose(poseEstimator.estimatedPosition)
            if (estimatedPose != null) {
                val timestamp = estimatedPose.timestampSeconds
                val visionMeasurement2d = estimatedPose.estimatedPose.toPose2d()
                poseEstimator.addVisionMeasurement(visionMeasurement2d, timestamp)
                currentPose = poseEstimator.estimatedPosition
            }
        }

        /*
        Updates the robot position based on movement and rotation from the pidgey and encoders.
         */
        poseEstimator.update(this.pidgeyRotation, this.modulePositions)

        field.robotPose = poseEstimator.estimatedPosition

        "Pidgey Yaw" log this.pidgeyYaw
        "Pidgey Rotation2D" log this.pidgeyRotation.degrees
        "Robot Pose" log field.robotPose
    }

    /**
     * Sets the drive speeds for the swerve modules.
     *
     * @param forwardSpeed The forward speed.
     * @param leftSpeed The left speed.
     * @param turnSpeed The turn speed.
     * @param isFieldOriented Whether the drive is field-oriented.
     */
    fun setDriveSpeeds(
        forwardSpeed: Double,
        leftSpeed: Double,
        turnSpeed: Double,
        isFieldOriented: Boolean = SwerveParameters.Thresholds.IS_FIELD_ORIENTED,
    ) {
        "Forward speed" log forwardSpeed
        "Left speed" log leftSpeed

        // Converts to a measure that the robot actually understands
        var speeds =
            if (isFieldOriented) {
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardSpeed,
                    leftSpeed,
                    turnSpeed,
                    this.pidgeyRotation,
                )
            } else {
                ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed)
            }

        speeds = ChassisSpeeds.discretize(speeds, 0.02)

        val newStates =
            SwerveParameters.PhysicalParameters.kinematics.toSwerveModuleStates(speeds)
        SwerveDriveKinematics.desaturateWheelSpeeds(newStates, MotorParameters.MAX_SPEED)

        this.moduleStates = newStates
    }

    /**
     * Rotation of the Pigeon2 IMU
     */
    val pidgeyRotation: Rotation2d
        get() = pidgey.rotation2d

    /**
     * Heading of the robot
     */
    val heading: Double
        get() = -pidgey.yaw.valueAsDouble

    /**
     * Yaw of the Pigeon2 IMU
     */
    val pidgeyYaw: Double
        get() = pidgey.yaw.valueAsDouble

    /** Resets the Pigeon2 IMU.  */
    fun resetPidgey() {
        pidgey.reset()
    }

    /**
     * The current pose of the robot from the pose estimator
     */
    val pose: Pose2d
        get() = poseEstimator.estimatedPosition

    /** Resets the pose of the robot to zero.  */
    fun zeroPose() {
        poseEstimator.resetPosition(
            Rotation2d.fromDegrees(this.heading),
            this.modulePositions,
            Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        )
    }

    /**
     * Sets a new pose for the robot.
     *
     * @param pose The new pose.
     */
    fun newPose(pose: Pose2d?) {
        poseEstimator.resetPosition(this.pidgeyRotation, this.modulePositions, pose)
    }

    /**
     * The chassis speeds, used for autonomous driving
     */
    val autoSpeeds: ChassisSpeeds
        get() {
            val k = SwerveParameters.PhysicalParameters.kinematics
            return k.toChassisSpeeds(*this.moduleStates)
        }

    /**
     * The rotation of the Pigeon2 IMU, used for PID control
     */
    val rotationPidgey: Rotation2d
        get() = Rotation2d.fromDegrees(-pidgey.rotation2d.degrees)

    /**
     * Drives the robot using chassis speeds.
     *
     * @param chassisSpeeds The chassis speeds.
     */
    fun chassisSpeedsDrive(chassisSpeeds: ChassisSpeeds?) {
        val newStates =
            SwerveParameters.PhysicalParameters.kinematics.toSwerveModuleStates(chassisSpeeds)
        this.moduleStates = newStates
    }

    /**
     * The states of the swerve modules
     */
    var moduleStates: Array<SwerveModuleState>
        get() {
            val moduleStates = emptyArray<SwerveModuleState>()
            for (i in modules.indices) {
                moduleStates[i] = modules[i].state
            }
            return moduleStates
        }

        set(states) {
            for (i in states.indices) {
                modules[i].state = states[i]
            }
        }

    /**
     * The positions of the swerve modules
     */
    val modulePositions: Array<SwerveModulePosition?>
        get() {
            val positions = arrayOfNulls<SwerveModulePosition>(states.size)
            for (i in positions.indices) {
                positions[i] = modules[i].position
            }
            return positions
        }

    /** Stops all swerve modules.  */
    fun stop() {
        for (module in modules) {
            module.stop()
        }
    }

    /** Sets the PID constants for autonomous driving.  */
    fun setAutoPID() {
        for (module in modules) {
            module.setAutoPID()
        }
    }

    /** Sets the PID constants for teleoperated driving.  */
    fun setTelePID() {
        for (module in modules) {
            module.setTelePID()
            module.applyTelePIDValues()
        }
    }

    /** Resets the drive positions of all swerve modules.  */
    fun resetDrive() {
        for (module in modules) {
            module.resetDrivePosition()
        }
    }

    fun updateModuleTelePIDValues() {
        for (module in modules) {
            module.updateTelePID()
        }
    }

    /**
     * Drive the robot using an Xbox controller in field-oriented mode.
     *
     * @param controller The Xbox controller to use for driving.
     */

    fun fieldDriveUsingController(controller : XboxController){
        var x: Double = -controller.leftX * MAX_SPEED
        if (abs(x) < X_DEADZONE * MAX_SPEED) x = 0.0
        x *= if (slowmode) 0.25 else 1.0

        var y: Double = -controller.leftY * MAX_SPEED
        if (abs(y) < Y_DEADZONE * MAX_SPEED) y = 0.0
        y *= if (slowmode) 0.25 else 1.0

        val rotation = if (abs(controller.rightX) >= 0.1) (-controller.rightX * MAX_ANGULAR_SPEED * if (slowmode) 0.0625 else 0.25) else 0.0

        // I dunno how logging works yet lol - Sam
//        logs {
//            log("Swerve/Drive/X Speed", x)
//            log("Swerve/Drive/Y Speed", y)
//            log("Swerve/Drive/Rotation Speed", rotation)
//            log("Swerve/Drive/Vision Dead", visionDead)
//            log("Swerve/Drive/Slow Mode Enabled", slowmode)
//        }

        setDriveSpeeds(y, x, rotation, true)
    }

    fun pathFindToGoal(): Command? = AutoBuilder.pathfindThenFollowPath(pathToScore, constraints)

    fun pathFindTest(): Command? = AutoBuilder.pathfindThenFollowPath(pathToScore, constraints)
}
