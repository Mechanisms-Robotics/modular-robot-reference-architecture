package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PoseEstimator8736;
import frc.robot.CONSTANTS.VisionConstants;

public class Vision extends SubsystemBase {
    private final PoseCameraIO[] ios;
    private final PoseCameraIOInputsAutoLogged[] inputs;

    private final PoseEstimator8736 poseEstimator;

    // The cameraName here is used for logging purposes
    public Vision(PoseEstimator8736 poseEstimator, PoseCameraIO... ios) {
        this.ios = ios;
        this.poseEstimator = poseEstimator;
        this.inputs = new PoseCameraIOInputsAutoLogged[ios.length];

        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new PoseCameraIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + i, inputs[i]);

            // Constantly feed vision measurements into the pose estimator
            for (int j = 0; j < inputs[i].timestampSeconds.length; j++) {
                // Sanity gate: the robot drives on the floor, so a solution
                // that puts it half a meter above OR below the carpet is a
                // bad tag solve. The old check only rejected above-floor
                // poses; Math.abs also catches below-floor ones.
                double z = inputs[i].poseEstimates[j].getZ();
                if (Math.abs(z) > VisionConstants.Z_THRESHOLD) {
                    continue;
                }

                this.poseEstimator.addVisionMeasurement(
                    inputs[i].poseEstimates[j].toPose2d(),
                    inputs[i].timestampSeconds[j],
                    VecBuilder.fill(0.9, 0.9, 0.9)
                );
            }
        }
    }
}
