package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.TimeSensitive;
import simulator.framework.CarButton;
public class CarButtonControl extends Controller implements TimeSensitive{
	SimTime period;
	CarButton localCarButton;

	public CarButtonControl(SimTime period, boolean verbose) {
		super("CarButtonControl", verbose);
		
		this.period = period;
	}

	@Override
	public void timerExpired(Object callbackData) {
		
	}

}
