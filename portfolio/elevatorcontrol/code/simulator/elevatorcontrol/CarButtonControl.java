package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.framework.CarButton;
import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.TimeSensitive;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
public class CarButtonControl extends Controller implements TimeSensitive{
	
	//local physical state
	private ReadableCarCallPayload localCarCall;
	private WriteableCarLightPayload localCarLight;
	
	//network interface
	
	//keep track of which instance this is
	private final Hallway hallway;
	private final int floor;
	
	private SimTime period;

	private enum State{
		STATE_IDLE,
		STATE_CARCALL_PLACED,
		STATE_CARCALL_SERVED,
	}

	private State state = State.STATE_IDLE;
	
	public CarButtonControl(int floor, Hallway hallway,SimTime period, boolean verbose) {
		super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor,hallway), verbose);
		
		//store constructor arguments in internal state	
		this.period = period;
		this.hallway = hallway;
		this.floor = floor;
		
		//log construction
		log("Created CarButtonControl floor ",floor,", hallway ",hallway);
		
		//initialize physical state
		//create a carcall payload object for this floor, hallway
		localCarCall = CarCallPayload.getReadablePayload(floor, hallway);
		
		//register the payload with the physical interface as input
		physicalInterface.registerTimeTriggered(localCarCall);
		
		//create a carlight payload object for this floor, hallway
		localCarLight = CarLightPayload.getWriteablePayload(floor, hallway);
		
		//register the carlight payload with the physical interface as output
		physicalInterface.sendTimeTriggered(localCarLight, period);
		
		timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		switch (state) {
		case STATE_IDLE:	
			localCarLight.set(false);

			//#transition 'T9.1'
			if(localCarCall.pressed()){
				newState = State.STATE_CARCALL_PLACED;
			}
			else{
				newState = state;
			}
			break;
		
		case STATE_CARCALL_PLACED:
			localCarLight.set(true);
			
			//#transition 'T9.2'
			if(!localCarCall.pressed() && mAtFloor.getValue() == true){
				newState = State.STATE_IDLE;
			}
			else{
				newState = state;
			}
			break;
			
		default:
            throw new RuntimeException("State " + state + " was not recognized.");

		}
		

	}

}
