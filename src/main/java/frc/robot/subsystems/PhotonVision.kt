package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.PhotonVisionConstants
import org.photonvision.EstimatedRobotPose
import org.photonvision.targeting.PhotonPipelineResult
import org.photonvision.targeting.PhotonTrackedTarget
import xyz.malefic.frc.pingu.LogPingu.log
import xyz.malefic.frc.pingu.LogPingu.logs
import xyz.malefic.frc.sub.PhotonModule
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * The PhotonVision class is a subsystem that interfaces with multiple PhotonVision cameras to
 * provide vision tracking and pose estimation capabilities. This subsystem is a Singleton that
 * manages multiple CameraModules and selects the best result based on pose ambiguity.
 *
 *
 * This subsystem provides methods to get the estimated global pose of the robot, the distance to
 * targets, and the yaw of detected AprilTags. It also provides methods to check if a tag is visible
 * and get the pivot position based on distance calculations.
 */
object PhotonVision : SubsystemBase() {
    private val cameras: MutableList<PhotonModule> = ArrayList()
    private var bestCamera: PhotonModule? = null
    private var currentResult: PhotonPipelineResult? = null

    /**
     * Gets the current tracked target.
     *
     * @return The current PhotonTrackedTarget, or null if no target is tracked
     */
    var currentTarget: PhotonTrackedTarget? = null
        private set

    /**
     * Gets the current yaw angle to the target.
     *
     * @return The yaw angle in degrees
     */
    var yaw: Double = -15.0
        private set
    var y: Double = 0.0
        private set
    var dist: Double = 0.0
        private set

    /**
     * Gets the current target pose ambiguity.
     *
     * @return The target pose ambiguity value
     */
    var targetPoseAmbiguity: Double = 7157.0
        private set

    /**
     * Creates a new instance of this PhotonVision subsystem. This constructor is private since this
     * class is a Singleton. Code should use the [.getInstance] method to get the singleton
     * instance.
     */
    init {
        // Initialize cameras with their positions
        val fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

        // First camera setup
        val camera1Pos =
            Transform3d(
                Translation3d(0.31, 0.0, PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
                Rotation3d(
                    0.0,
                    Math.toRadians(360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG),
                    Math.toRadians(180.0),
                ),
            )
        cameras.add(PhotonModule("Camera", camera1Pos, fieldLayout))

        // Add additional cameras here as needed
    }

    /**
     * This method is called periodically by the CommandScheduler. It updates the tracked targets,
     * selects the best camera based on pose ambiguity, and updates dashboard information.
     */
    override fun periodic() {
        updateBestCamera()
        if (bestCamera == null) return

        val results: MutableList<PhotonPipelineResult> = bestCamera!!.allUnreadResults
        currentResult = if (results.isEmpty()) null else results[0]

        if (currentResult == null) return

        currentTarget = currentResult!!.bestTarget
        targetPoseAmbiguity = if (currentTarget != null) currentTarget!!.getPoseAmbiguity() else 7157.0

        for (tag in currentResult!!.getTargets()) {
            yaw = tag.getYaw()
            y = tag.getBestCameraToTarget().x
            dist = tag.getBestCameraToTarget().z
        }

        // Update dashboard
        logs {
            log("yaw to target", yaw)
            log("cam ambiguity", targetPoseAmbiguity)
            log("_targets", currentResult!!.hasTargets())
        }
    }

    /** Updates the best camera selection based on pose ambiguity of detected targets.  */
    private fun updateBestCamera() {
        bestCamera = this.cameraWithLeastAmbiguity
    }

    private val cameraWithLeastAmbiguity: PhotonModule?
        /**
         * Selects the camera with the least pose ambiguity from all available cameras.
         *
         * @return The CameraModule with the lowest pose ambiguity, or null if no cameras have valid
         * targets
         */
        get() {
            var bestCam: PhotonModule? = null
            var bestAmbiguity = Double.MAX_VALUE

            for (camera in cameras) {
                val results: MutableList<PhotonPipelineResult> = camera.allUnreadResults
                if (results.isEmpty()) continue
                for (result in results) {
                    if (result.hasTargets()) {
                        val target = result.bestTarget
                        if (target != null && target.getPoseAmbiguity() < bestAmbiguity) {
                            bestAmbiguity = target.getPoseAmbiguity()
                            bestCam = camera
                        }
                    }
                }
            }

            return bestCam
        }

    /**
     * Checks if there is a visible AprilTag.
     *
     *
     * This method is useful to avoid NullPointerExceptions when trying to access specific info
     * based on vision.
     *
     * @return true if there is a visible tag, false otherwise
     */
    fun hasTag(): Boolean = currentResult != null && currentResult!!.hasTargets()

    /**
     * Gets the estimated global pose of the robot using the best available camera.
     *
     * @param prevEstimatedRobotPose The previous estimated pose of the robot
     * @return The estimated robot pose, or null if no pose could be estimated
     */
    fun getEstimatedGlobalPose(prevEstimatedRobotPose: Pose2d?): EstimatedRobotPose? {
        if (bestCamera == null) return null

        val estimator = bestCamera!!.poseEstimator
        estimator.setReferencePose(prevEstimatedRobotPose)
        return if (currentResult != null) estimator.update(currentResult).orElse(null) else null
    }

    val estimatedGlobalPose: Transform3d
        /**
         * Gets the estimated global pose of the robot as a [Transform3d].
         *
         * @return The estimated global pose as a [Transform3d]
         */
        get() {
            if (currentResult == null || currentResult!!.multiTagResult.isEmpty) {
                return Transform3d(0.0, 0.0, 0.0, Rotation3d())
            }
            return currentResult!!
                .multiTagResult
                .get()
                .estimatedPose.best
        }

    val distanceToAprilTag: Double
        /**
         * Calculates the straight-line distance to the currently tracked AprilTag.
         *
         * @return The distance to the AprilTag in meters
         */
        get() {
            val pose = this.estimatedGlobalPose
            return sqrt(
                pose.translation.x.pow(2.0) + pose.translation.y.pow(2.0),
            )
        }
}
