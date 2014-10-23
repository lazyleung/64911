package simulator.elevatorcontrol;

import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;

public class RuntimeRequirementsMonitor extends RuntimeMonitor  {
	
	RT6StateMachine RT6State = new RT6StateMachine();
	RT7StateMachine RT7State = new RT7StateMachine();
	boolean[][] carCalls = new boolean[Elevator.numFloors][2];
	boolean[][] hallCalls = new boolean[Elevator.numFloors][2];	
    protected int currentFloor = MessageDictionary.NONE;
    protected Hallway currentHallway = Hallway.NONE;

	@Override
	public void timerExpired(Object callbackData) {
		//do nothing
		
	}

	@Override
	protected String[] summarize() {
		//do nothing
		return null;
	}
	
    public void receive(ReadableAtFloorPayload msg) {
        updateCurrentFloor(msg);
    }
	
	public void receive(ReadableDriveSpeedPayload msg) {
		RT6State.receive(msg);
		
	}
	
	public void receive(ReadableDoorClosedPayload msg) {
		RT7State.receive(msg);
		}
	
    private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
        if (lastAtFloor.getFloor() == currentFloor) {
            //the atFloor message is for the currentfloor, so check both sides to see if they a
            if (!atFloors[lastAtFloor.getFloor()-1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor()-1][Hallway.FRONT.ordinal()].value()) {
                //both sides are false, so set to NONE
                currentFloor = MessageDictionary.NONE;
                currentHallway = Hallway.NONE;
            }
            //otherwise at least one side is true, so leave the current floor as is
        } else {
            if (lastAtFloor.value()) {
                currentFloor = lastAtFloor.getFloor();
                currentHallway = lastAtFloor.getHallway();
            }
        }
    }

	
	
	
	
	private static enum RT6States{
		MOVING,
		STOPPED,
		STOPPED_NO_PENDING_CALLS
	}
	
	private static enum RT7States{
		DOORS_CLOSED,
		DOORS_OPEN,
		DOORS_OPEN_NO_PENDING_CALLS
	}
	
	
	private class RT6StateMachine{
		RT6States state;
		boolean warningIssued = false;
		
		public RT6StateMachine(){
			state = RT6States.STOPPED;
		}		
		
		public void receive(ReadableDriveSpeedPayload msg){
			updateState(msg);
		}
		
		private void updateState(ReadableDriveSpeedPayload msg){
			RT6States previousState = state;
			RT6States newState = previousState;
			
			if(msg.speed()!=0){
				newState = RT6States.MOVING;
			}
			else if(msg.speed()==0 && (carLights[currentFloor-1][currentHallway.ordinal()].lighted()||
        			hallLights[currentFloor-1][currentHallway.ordinal()][0].lighted() ||
        			hallLights[currentFloor-1][currentHallway.ordinal()][1].lighted())){
				newState = RT6States.STOPPED;				
			}
			else if(msg.speed()==0 && (!carLights[currentFloor-1][currentHallway.ordinal()].lighted()&&
        			!hallLights[currentFloor-1][currentHallway.ordinal()][0].lighted() &&
        			!hallLights[currentFloor-1][currentHallway.ordinal()][1].lighted())){
				newState = RT6States.STOPPED_NO_PENDING_CALLS;	
				warningIssued = false;
		}
			
			if(newState != previousState){
				switch(newState){
				case MOVING:
					break;
				case STOPPED:
					break;
				case STOPPED_NO_PENDING_CALLS:
					if(!warningIssued){
						warning("R-T.6 Violated: Car stopped at Floor " + currentFloor + " when there is no pending calls");
						warningIssued = true;
					}
					break;

				}
			}
			
		
		}
	}
	
	private class RT7StateMachine{
		RT7States state;
		boolean warningIssued = false;
		
		public RT7StateMachine(){
			state = RT7States.DOORS_CLOSED;
		}
		
        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg);
        }
        
        private void updateState(ReadableDoorClosedPayload msg){        	
        	RT7States previousState = state;
        	RT7States newState = previousState;
        	
        	if(msg.isClosed()){
        		newState = RT7States.DOORS_CLOSED;
        	}
        	else if(!msg.isClosed() && (carLights[currentFloor-1][currentHallway.ordinal()].lighted() ||
        			hallLights[currentFloor-1][currentHallway.ordinal()][0].lighted() ||
        			hallLights[currentFloor-1][currentHallway.ordinal()][1].lighted())){
        		//For now direction does not matter. as long as there is a hall call placed the car can stop at that hallway.
        		newState = RT7States.DOORS_OPEN;
        	}
        	else if (!msg.isClosed() && (!carLights[currentFloor-1][currentHallway.ordinal()].lighted() &&
        			!hallLights[currentFloor-1][currentHallway.ordinal()][0].lighted()&&
        			!hallLights[currentFloor-1][currentHallway.ordinal()][1].lighted())){
        		//For now direction does not matter. as long as there is a hall call placed the car can stop at that hallway.
        		newState = RT7States.DOORS_OPEN_NO_PENDING_CALLS;
				warningIssued = false;
        	}
        	
        	if(newState != previousState){
        		switch(newState){
        		case DOORS_CLOSED:
        			break;
        		case DOORS_OPEN:
        			break;
        		case DOORS_OPEN_NO_PENDING_CALLS:
        			if(!warningIssued){
        				warning("R-T.7 Violated: Car doors open at Floor " + currentFloor+ " "+ currentHallway + " when there is no pending calls");
        				warningIssued = true;
        			}
        			break;
        		}
        	}
        	
        }
	}
}
