package frc.utils;

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(2.0, 2.0, 2.0);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final Supplier<SwerveModuleState[]> moduleStateSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final PhotonRunnable photonEstimator = new PhotonRunnable();
  private final Notifier photonNotifier = new Notifier(photonEstimator);

  private OriginPosition originPosition = kBlueAllianceWallRightSide;
  private boolean sawTag = false;

  public PoseEstimatorSubsystem(DriveSubsystem m_drive) {
    
    this.rotationSupplier = m_drive::newHeading;
    this.modulePositionSupplier = m_drive::getModulePositions;
    this.moduleStateSupplier = m_drive::getModuleStates;

    poseEstimator =  new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    
    // Start PhotonVision thread
    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getCurrentPose, // Robot pose supplier
            this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (ChassisSpeeds speeds) -> m_drive.drive( // needs to be robot-relative
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            false, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.0, 0.0, 0.01), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                    0.75, // Max module speed, in m/s
                    0.54, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
                    //new ReplanningConfig(true, true, 3.0, 1.0) // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            m_drive // Reference to this subsystem to set requirements
    );
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    boolean allianceChanged = false;
    switch(alliance) {
      case Blue:
        allianceChanged = (originPosition == kRedAllianceWallRightSide);
        originPosition = kBlueAllianceWallRightSide;
        break;
      case Red:
        allianceChanged = (originPosition == kBlueAllianceWallRightSide);
        originPosition = kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }

    if (allianceChanged && sawTag) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag was seen, and the tags are all relative to the coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(getCurrentPose());
      poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

    var visionPose = photonEstimator.grabLatestEstimatedPose();
    if (visionPose != null) {
      // New pose from vision
      sawTag = true;
      var pose2d = visionPose.estimatedPose.toPose2d();
      if (originPosition != kBlueAllianceWallRightSide) {
        pose2d = flipAlliance(pose2d);
      }
      poseEstimator.addVisionMeasurement(pose2d, visionPose.timestampSeconds);
    }

    // Set the pose on the dashboard
    var dashboardPose = poseEstimator.getEstimatedPosition();
    if (originPosition == kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
    SmartDashboard.putData("Field", field2d);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public PhotonPipelineResult getLatestTag() {
    return photonEstimator.grabLatestTag();
  }

  public Pose2d getCurrentResultPose() {
    return photonEstimator.grabLatestResult();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStateSupplier.get());
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
  }

}