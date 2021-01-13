package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase; //highlight stages instead

public class Lighting extends SubsystemBase {
	private int rainbowHue = 0;

	private AddressableLED LED = new AddressableLED(1);
	private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(100);

	private int length = LEDBuffer.getLength();
	private int pulseIndex = 0;

	public Lighting() {
		LED.setLength(length);

		pulseBlue();

		LED.setData(LEDBuffer);
		LED.start();
	}

	public void pulseBlue() {
		for (int i = 0; i < length; i++) {
			LEDBuffer.setRGB(i, (int) (Math.sin(i + pulseIndex) * 14 + 14), (int) (Math.sin(i + pulseIndex) * 35 + 35), (int) (Math.sin(i + pulseIndex) * 71 + 71));
		}
		LED.setData(LEDBuffer);
		pulseIndex++;
	}

	public void rainbow() {
		for (var i = 0; i < LEDBuffer.getLength(); i++) {
			final var hue = (rainbowHue + (i * 180 / LEDBuffer.getLength())) % 180;
			LEDBuffer.setHSV(i, hue, 255, 128);
		}
		rainbowHue += 3;
		rainbowHue %= 180;
	}

	public void setRGB(int R, int G, int B, int start, int end) {
		for (var i = start; i < end; i++) {
			LEDBuffer.setRGB(i, R, G, B);
		}
		LED.setData(LEDBuffer);
	}

	@Override
	public void periodic() {
	}
}