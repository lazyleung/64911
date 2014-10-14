/*
ECE649 FALL 2014
Group 11
Jonathan Leung/jkleung1
Eric Newhall/enewhall
Mengzhe Li/mzli 
Ting Xu/tingx
*/
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.TimeSensitive;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class CarPositionControl extends Controller implements TimeSensitive{

	//local physical state
	private WriteableCarPositionIndicatorPayload localCarPositionIndicator;
	
	//network interface
	//outgoing messages
	private WriteableCanMailbox networkCarPositionIndicatorOut;	
	private IntegerCanPayloadTranslator mCarPositionIndicator;
	
	//incoming messages
	private ReadableCanMailbox[] networkAtFloorIns = new ReadableCanMailbox[10];
	private AtFloorCanPayloadTranslator[] mAtFloors = new AtFloorCanPayloadTranslator[10];
	
	//internal variables
	private int [] AtFloorFloors = {1, 1, 2, 3, 4, 5, 6, 7, 7, 8};
    private Hallway[] AtFloorHallways = {Hallway.FRONT, Hallway.BACK, Hallway.BACK, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.FRONT, Hallway.BACK, Hallway.FRONT};

	private int floor;
	private SimTime period;
	
	private enum State{
		STATE_IDLE,
		STATE_CAR_AT_FLOOR,
	}
	
	private State state = State.STATE_IDLE;
	
	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);
		
		this.period = period;
		this.floor = 0;
		
		//log construction
		log("Created CarPositionControl");
		
		//initialize physical state
		//create carpositionindicator payload object
		localCarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload(); 
		
		//register the payload with the physical interface as output
		physicalInterface.sendTimeTriggered(localCarPositionIndicator,period);
		
		//initialize network interface
		//create CAN mailboxe for mCarPositionIndicator
		networkCarPositionIndicatorOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_POSITION_CAN_ID);
		mCarPositionIndicator = new IntegerCanPayloadTranslator(networkCarPositionIndicatorOut);
		canInterface.sendTimeTriggered(networkCarPositionIndicatorOut, period);
		
		//create the AtFloor mailboxes
		for (int i = 0; i < mAtFloors.length; i++){
			networkAtFloorIns[i] = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID+ReplicationComputer.computeReplicationId(AtFloorFloors[i], AtFloorHallways[i]));
            mAtFloors[i] = new AtFloorCanPayloadTranslator(networkAtFloorIns[i], AtFloorFloors[i], AtFloorHallways[i]);
            canInterface.registerTimeTriggered(networkAtFloorIns[i]);		
		}
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		switch(state) {
		case STATE_IDLE:
			localCarPositionIndicator.set(floor);
			mCarPositionIndicator.set(floor);
			newState = State.STATE_IDLE;

			for(int i = 0; i<mAtFloors.length; i++){
				//#transition T10.1
				if(mAtFloors[i].getValue()){
					floor = AtFloorFloors[i];
					newState = State.STATE_CAR_AT_FLOOR;
					break;
				}
			}
			break;
		case STATE_CAR_AT_FLOOR:
			localCarPositionIndicator.set(floor);
			mCarPositionIndicator.set(floor);
			//#transition T10.2
			newState = State.STATE_IDLE;
			break;
		
		default:
        throw new RuntimeException("State " + state + " was not recognized.");
		}
		//log the results of this iteration
        if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",newState);
        }
        //update the state variable
        state = newState;
		//report the current state
        setState(STATE_KEY, newState.toString());

        //schedule the next iteration of the controller	
        timer.start(period);

	}

}
