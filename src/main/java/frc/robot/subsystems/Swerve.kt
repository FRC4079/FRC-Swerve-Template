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
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.MotorParameters
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.SHOULD_INVERT
import org.photonvision.EstimatedRobotPose
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.pingu.Pingu
import java.util.function.BooleanSupplier

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
                logs("Swerve Module States", *this.moduleStates)
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
                logs("Set Swerve Module States", *setStates)
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

    /**
     * Creates a new instance of this SwerveSubsystem. This constructor is private since this class is
     * a Singleton. Code should use the [.getInstance] method to get the singleton instance.
     */
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

        logs {
            log("Pidgey Yaw", this.pidgeyYaw)
            log("Pidgey Rotation2D", this.pidgeyRotation!!.degrees)
            log("Robot Pose", field.robotPose)
        }
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
        log("Forward speed", forwardSpeed)
        log("Left speed", leftSpeed)

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

    val pidgeyRotation: Rotation2d?
        /**
         * Gets the rotation of the Pigeon2 IMU.
         *
         * @return Rotation2D, The rotation of the Pigeon2 IMU.
         */
        get() = pidgey.rotation2d

    val heading: Double
        /**
         * Gets the heading of the robot.
         *
         * @return double, The heading of the robot.
         */
        get() = -pidgey.yaw.valueAsDouble

    val pidgeyYaw: Double
        /**
         * Gets the yaw of the Pigeon2 IMU.
         *
         * @return double, The yaw of the Pigeon2 IMU.
         */
        get() = pidgey.yaw.valueAsDouble

    /** Resets the Pigeon2 IMU.  */
    fun resetPidgey() {
        pidgey.reset()
    }

    val pose: Pose2d?
        /**
         * Gets the current pose of the robot from the pose estimator.
         *
         * @return The current pose of the robot.
         */
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

    val autoSpeeds: ChassisSpeeds?
        /**
         * Gets the chassis speeds for autonomous driving.
         *
         * @return ChassisSpeeds, The chassis speeds for autonomous driving.
         */
        get() {
            val k = SwerveParameters.PhysicalParameters.kinematics
            return k.toChassisSpeeds(*this.moduleStates)
        }

    val rotationPidgey: Rotation2d
        /**
         * Gets the rotation of the Pigeon2 IMU for PID control.
         *
         * @return Rotation2D, The rotation of the Pigeon2 IMU for PID control.
         */
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

    var moduleStates: Array<SwerveModuleState>
        /**
         * Gets the states of the swerve modules.
         *
         * @return SwerveModuleState[], The states of the swerve modules.
         */
        get() {
            val moduleStates = emptyArray<SwerveModuleState>()
            for (i in modules.indices) {
                moduleStates[i] = modules[i].state
            }
            return moduleStates
        }

        /**
         * Sets the states of the swerve modules.
         *
         * @param states The states of the swerve modules.
         */
        set(states) {
            for (i in states.indices) {
                modules[i].state = states[i]
            }
        }

    val modulePositions: Array<SwerveModulePosition?>
        /**
         * Gets the positions of the swerve modules.
         *
         * @return SwerveModulePosition[], The positions of the swerve modules.
         */
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

    fun pathFindToGoal(): Command? = AutoBuilder.pathfindThenFollowPath(pathToScore, constraints)

    fun pathFindTest(): Command? = AutoBuilder.pathfindThenFollowPath(pathToScore, constraints)
}
