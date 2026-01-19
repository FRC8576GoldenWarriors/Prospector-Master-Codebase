// package frc.robot.util;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import frc.robot.subsystems.vision.VisionConstants;
// import frc.robot.subsystems.vision.VisionIO.PoseObservation;
// import frc.robot.subsystems.vision.VisionIO.PoseObservationType;

// public class PoseUtil {

//     public static PoseObservation fuseEstimates(GoldenSwervePoseEstimator poseEstimator, PoseObservation poseObservationA, PoseObservation poseObservationB) {
//         if(poseObservationA.timestamp() < poseObservationB.timestamp()) {
//             var temp = poseObservationA;
//             poseObservationA = poseObservationB;
//             poseObservationB = temp;
//         }

//         Transform2d poseA_To_poseB
//             =  poseEstimator.sampleAt(poseObservationA.timestamp())
//             .get()
//             .minus(poseEstimator.sampleAt(poseObservationB.timestamp()).get());

//         Pose2d poseA2d = poseObservationA.pose().toPose2d();
//         Pose2d poseB2d = poseObservationB.pose().toPose2d();

//         Pose2d newPoseA = poseA2d.transformBy(poseA_To_poseB);
//         Pose2d newPoseB = poseB2d;

//         Matrix<N3, N1> poseObvASTD = VecBuilder.fill(poseObservationA.standardDeviations()[0], poseObservationA.standardDeviations()[1], poseObservationA.standardDeviations()[2]);
//         Matrix<N3, N1> poseObvBSTD = VecBuilder.fill(poseObservationB.standardDeviations()[0], poseObservationB.standardDeviations()[1], poseObservationB.standardDeviations()[2]);

//         Matrix<N3, N1> varianceA = poseObvASTD.elementTimes(poseObvASTD);
//         Matrix<N3, N1> varianceB = poseObvASTD.elementTimes(poseObvBSTD);

//         Rotation2d fusedHeading = newPoseB.getRotation();
//         if (varianceA.get(2, 0) < VisionConstants.maxAmbiguity
//                 && varianceB.get(2, 0) < VisionConstants.maxAmbiguity) {
//             fusedHeading =
//                     new Rotation2d(
//                             newPoseA.getRotation().getCos() / varianceA.get(2, 0)
//                                     + newPoseB.getRotation().getCos() / varianceB.get(2, 0),
//                             newPoseA.getRotation().getSin() / varianceA.get(2, 0)
//                                     + newPoseB.getRotation().getSin() / varianceB.get(2, 0));
//         }

//         double weightAx = 1.0 / varianceA.get(0, 0);
//         double weightAy = 1.0 / varianceA.get(1, 0);
//         double weightBx = 1.0 / varianceB.get(0, 0);
//         double weightBy = 1.0 / varianceB.get(1, 0);

//         Pose2d fusedPose =
//                 new Pose2d(
//                         new Translation2d(
//                                 (newPoseA.getTranslation().getX() * weightAx
//                                                 + newPoseB.getTranslation().getX() * weightBx)
//                                         / (weightAx + weightBx),
//                                 (newPoseA.getTranslation().getY() * weightAy
//                                                 +newPoseB.getTranslation().getY() * weightBy)
//                                         / (weightAy + weightBy)),
//                         fusedHeading);

//         double[] fusedStdDev =
//                 {
//                         Math.sqrt(1.0 / (weightAx + weightBx)),
//                         Math.sqrt(1.0 / (weightAy + weightBy)),
//                         Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0)))};

//         int numTags = poseObservationA.tagCount() + poseObservationB.tagCount();
//         double time = poseObservationB.timestamp();
//         double ambiguity = 1/Math.sqrt(1/Math.pow(poseObservationA.ambiguity(), 2) + 1/1/Math.pow(poseObservationA.ambiguity(), 2));
//         double averageTagDistance = (poseObservationA.averageTagDistance() * poseObservationA.tagCount() + poseObservationB.averageTagDistance() * poseObservationB.tagCount()) / numTags;


//         return new PoseObservation(time, new Pose3d(fusedPose), ambiguity, numTags, averageTagDistance, fusedStdDev, (poseObservationA.type() == PoseObservationType.MEGATAG_2) ? PoseObservationType.MEGATAG_2 : PoseObservationType.MEGATAG_1);
//     }

// }
