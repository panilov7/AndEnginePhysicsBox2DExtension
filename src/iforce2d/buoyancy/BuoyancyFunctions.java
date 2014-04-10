package iforce2d.buoyancy;

import com.badlogic.gdx.physics.box2d.Fixture;

public class BuoyancyFunctions {

	static {
		System.loadLibrary("andenginephysicsbox2dextension");
	}

	/**
	 * This method makes everything work. Basically you give it the sensor
	 * fixture and the fixture that should float and it does all the magic.
	 * 
	 * @param senceFix
	 *            - The fluid/sensor fixture
	 * @param bodyFix
	 *            - The fixture that is being buoyant
	 */
	public static void makeBuoyant(Fixture senceFix, Fixture bodyFix) {
		doWork(senceFix.getAddr(), bodyFix.getAddr());
	}

	private native static void doWork(long fA, long fB);

}