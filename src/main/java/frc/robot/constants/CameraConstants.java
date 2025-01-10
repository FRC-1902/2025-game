package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class CameraConstants {
    public static final double MAXIMUM_AMBIGUITY = 0.25; // TODO: Adjust later

    // TODO: Set real values
	public enum Camera {
		ArducamOne(
				"ArducamOne",
				new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(30)),
				new Translation3d(
						Units.inchesToMeters(12.056), Units.inchesToMeters(10.981), Units.inchesToMeters(8.44)),
				VecBuilder.fill(4, 4, 8),
				VecBuilder.fill(0.5, 0.5, 1)),
        ArducamTwo(
				"ArducamTwo",
				new Rotation3d(0, Math.toRadians(-24.094), Math.toRadians(-30)),
				new Translation3d(
						Units.inchesToMeters(12.056),
						Units.inchesToMeters(-10.981),
						Units.inchesToMeters(8.44)),
				VecBuilder.fill(4, 4, 8),
				VecBuilder.fill(0.5, 0.5, 1)),
        ArducamThree(
				"ArducamThree",
				new Rotation3d(0, Units.degreesToRadians(-145), 0),
				new Translation3d(
						Units.inchesToMeters(-4.628),
						Units.inchesToMeters(-10.687),
						Units.inchesToMeters(16.129)),
				VecBuilder.fill(4, 4, 8),
				VecBuilder.fill(0.5, 0.5, 1)),
        ArducamFour(
				"ArducamFour",
				new Rotation3d(0, Units.degreesToRadians(-145), 0),
				new Translation3d(
						Units.inchesToMeters(-4.628),
						Units.inchesToMeters(-10.687),
						Units.inchesToMeters(16.129)),
				VecBuilder.fill(4, 4, 8),
				VecBuilder.fill(0.5, 0.5, 1));

		public final String name;
		public final Rotation3d rotation;
		public final Translation3d translation;
		public final Matrix<N3, N1> singleTagStdDevs;
		public final Matrix<N3, N1> multiTagStdDevs;

		Camera(
            String name,
            Rotation3d rotation,
            Translation3d translation,
            Matrix<N3, N1> singleTagStdDevs,
            Matrix<N3, N1> multiTagStdDevs) {
                this.name = name;
                this.rotation = rotation;
                this.translation = translation;
                this.singleTagStdDevs = singleTagStdDevs;
                this.multiTagStdDevs = multiTagStdDevs;
		    }
	}

	public static final Pose3d[] CAMERA_POSITIONS = {
		new Pose3d(Camera.ArducamOne.translation, Camera.ArducamOne.rotation),
		new Pose3d(Camera.ArducamTwo.translation, Camera.ArducamTwo.rotation),
		new Pose3d(Camera.ArducamThree.translation, Camera.ArducamThree.rotation),
		new Pose3d(Camera.ArducamFour.translation, Camera.ArducamFour.rotation)
	};
}
