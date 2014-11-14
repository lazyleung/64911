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
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;
public class CarButtonControl extends Controller implements TimeSensitive{
	
	//local physical state
	private ReadableCarCallPayload localCarCall;
	private WriteableCarLightPayload localCarLight;
	
	//network interface
	private WriteableCanMailbox networkCarCallOut;
    private CarCallCanPayloadTranslator mCarCall;

    //incoming messages
	private ReadableCanMailbox networkAtFloorIn;
    private AtFloorCanPayloadTranslator mAtFloor;

	
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
	
	public CarButtonControl(SimTime period, int floor, Hallway hallway, boolean verbose) {
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
		
		//initialize network interface
		//create CAN mailboxes
		networkCarCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		networkAtFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));

		//Create a translator with a reference to the CanMailbox.  Use the 
        //translator to read and write values to the mailbox
		
		mCarCall = new CarCallCanPayloadTranslator(networkCarCallOut, floor, hallway);
		mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloorIn, floor, hallway);
		
        //register mailboxes to have its value broadcasted/receive updates on the network periodically
        canInterface.sendTimeTriggered(networkCarCallOut, period);
        canInterface.registerTimeTriggered(networkAtFloorIn);


        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers.
         */		
		timer.start(period);
		
	}

	public CarButtonControl(int floor, Hallway hallway, SimTime period, boolean verbose) {
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
		
		//initialize network interface
		//create CAN mailboxes
		networkCarCallOut = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		networkAtFloorIn = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));

		//Create a translator with a reference to the CanMailbox.  Use the 
        //translator to read and write values to the mailbox
		
		mCarCall = new CarCallCanPayloadTranslator(networkCarCallOut, floor, hallway);
		mAtFloor = new AtFloorCanPayloadTranslator(networkAtFloorIn, floor, hallway);
		
        //register mailboxes to have its value broadcasted/receive updates on the network periodically
        canInterface.sendTimeTriggered(networkCarCallOut, period);
        canInterface.registerTimeTriggered(networkAtFloorIn);


        /* issuing the timer start method with no callback data means a NULL value 
         * will be passed to the callback later.  Use the callback data to distinguish
         * callbacks from multiple calls to timer.start() (e.g. if you have multiple
         * timers.
         */		
		timer.start(period);
		
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		switch (state) {
			case STATE_IDLE:	
				localCarLight.set(false);
				mCarCall.set(false);

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
				mCarCall.set(true);

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
