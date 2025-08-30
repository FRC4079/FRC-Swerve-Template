package frc.robot.subsystems

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Transform3d
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator
import org.photonvision.targeting.PhotonPipelineResult

/**z
 * The CameraModule class represents a single Photonvision camera setup with its associated pose
 * estimator and position information. This class encapsulates all the functionality needed for a
 * single camera to track AprilTags and estimate robot pose.
 */
class PhotonModule(
    cameraName: String?,
    cameraPos: Transform3d?,
    fieldLayout: AprilTagFieldLayout?,
) {
    private val camera: PhotonCamera

    /**
     * Gets the pose estimator associated with this camera.
     *
     * @return PhotonPoseEstimator, The PhotonPoseEstimator object used for robot pose estimation
     */
    val poseEstimator: PhotonPoseEstimator

    /**
     * Gets the camera's position relative to the robot.
     *
     * @return Transform3d, The Transform3d representing the camera's position
     */
    val cameraPosition: Transform3d?

    /**
     * Creates a new CameraModule with the specified parameters.git p
     *
     * @param cameraName The name of the camera in the Photonvision interface
     * @param cameraPos The 3D transform representing the camera's position relative to the robot
     * @param fieldLayout The AprilTag field layout used for pose estimation
     */
    init {
        this.camera = PhotonCamera(cameraName)
        this.cameraPosition = cameraPos
        this.poseEstimator =
            PhotonPoseEstimator(
                fieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraPos,
            )
    }

    val allUnreadResults: MutableList<PhotonPipelineResult?>?
        /**
         * Gets all unread pipeline results from the camera.
         *
         * @return A list of PhotonPipelineResult objects containing the latest vision processing results
         */
        get() = camera.getAllUnreadResults()
}
