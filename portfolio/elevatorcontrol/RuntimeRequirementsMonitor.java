package simulator.elevatorcontrol;

import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;

public class RuntimeRequirementsMonitor extends RuntimeMonitor  {
	
	RT6StateMachine RT6State = new RT6StateMachine();
	RT7StateMachine RT7State = new RT7StateMachine();
	RT10StateMachine RT10State = new RT10StateMachine();
	
	boolean[][] carCalls = new boolean[Elevator.numFloors][2];
	boolean[][] hallCalls = new boolean[Elevator.numFloors][2];	
    protected int currentFloor = MessageDictionary.NONE;
    protected Hallway currentHallway = Hallway.NONE;
    protected boolean reversal [][] = new boolean[2][2];
    
	@Override
	public void timerExpired(Object callbackData) {
		//do nothing		
	}

	@Override
	protected String[] summarize() {
		//no outputs
		String[] arr = new String[1];
		return arr;
	}
	
    public void receive(ReadableAtFloorPayload msg) {
        updateCurrentFloor(msg);
    }
    
    public void receive(ReadableDoorReversalPayload msg) {
    	setReversal(msg);
    }
	
	public void receive(ReadableDriveSpeedPayload msg) {
		RT6State.receive(msg);	
		unsetReversal(msg);
	}
	
	public void receive(ReadableDoorClosedPayload msg) {
		RT7State.receive(msg);
	}
	
	public void receive(ReadableDoorMotorPayload msg){
		RT10State.receive(msg);
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
	
    private void setReversal(ReadableDoorReversalPayload msg){
    	if(msg.isReversing()){
    		reversal[msg.getHallway().ordinal()][msg.getSide().ordinal()] = true;
    	}    	
    }
    
    private void unsetReversal(ReadableDriveSpeedPayload msg){
    	if(msg.speed()!=0){
    		reversal[Hallway.BACK.ordinal()][Side.LEFT.ordinal()] = false;
    		reversal[Hallway.FRONT.ordinal()][Side.LEFT.ordinal()] = false;
			reversal[Hallway.BACK.ordinal()][Side.RIGHT.ordinal()] = false;
			reversal[Hallway.FRONT.ordinal()][Side.RIGHT.ordinal()] = false;
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
	
	private static enum RT10States{
		DOORS_STOPPED,
		DOOR_NUDGING_AFTER_REVERSAL,
		DOORS_NUDGING_NO_REVERSAL
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
			else if(msg.speed()==0 && currentFloor>0 &&currentFloor<=8 && (carLights[currentFloor-1][0].lighted()||
					carLights[currentFloor-1][1].lighted()||
        			hallLights[currentFloor-1][0][0].lighted() ||
        			hallLights[currentFloor-1][1][0].lighted() ||
        			hallLights[currentFloor-1][0][1].lighted() ||
        			hallLights[currentFloor-1][1][1].lighted())){
				newState = RT6States.STOPPED;				
			}
			else if(msg.speed()==0 && currentFloor>0 &&currentFloor<=8 && (!carLights[currentFloor-1][0].lighted()&&
					!carLights[currentFloor-1][1].lighted()&&
        			!hallLights[currentFloor-1][0][0].lighted() &&
        			!hallLights[currentFloor-1][1][0].lighted() &&
        			!hallLights[currentFloor-1][0][1].lighted() &&
        			!hallLights[currentFloor-1][1][1].lighted())){
				newState = RT6States.STOPPED_NO_PENDING_CALLS;	
		}
			
			if(newState != previousState){
				switch(newState){
				case MOVING:
					warningIssued = false;
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
			
			state = newState;
		
		}
	}
	
	private class RT7StateMachine{
		RT7States state;
		
		public RT7StateMachine(){
			state = RT7States.DOORS_CLOSED;
		}
		
        public void receive(ReadableDoorClosedPayload msg) {
            if(!msg.isClosed()){
            	//warning("door open!");
            }
            updateState(msg);

        }
        
        private void updateState(ReadableDoorClosedPayload msg){        	
        	RT7States previousState = state;
        	RT7States newState = previousState;
        	
        	if(msg.isClosed()){
        		newState = RT7States.DOORS_CLOSED;
        	}
        	else if(!msg.isClosed() && currentFloor>0 &&currentFloor<=8 && (carLights[currentFloor-1][msg.getHallway().ordinal()].lighted() ||
        			hallLights[currentFloor-1][msg.getHallway().ordinal()][0].lighted() ||
        			hallLights[currentFloor-1][msg.getHallway().ordinal()][1].lighted())){
        			//For now direction does not matter. as long as there is a hall call placed the car can stop at that hallway.
        			newState = RT7States.DOORS_OPEN;
        	}
        	else if (!msg.isClosed() && currentFloor>0 &&currentFloor<=8 && (!carLights[currentFloor-1][msg.getHallway().ordinal()].lighted() &&
        			!hallLights[currentFloor-1][msg.getHallway().ordinal()][0].lighted()&&
        			!hallLights[currentFloor-1][msg.getHallway().ordinal()][1].lighted())){
        			//For now direction does not matter. as long as there is a hall call placed the car can stop at that hallway.
        			newState = RT7States.DOORS_OPEN_NO_PENDING_CALLS;
        	}
        	
        	if(newState != previousState){
        		switch(newState){
        		case DOORS_CLOSED:
        			break;
        		case DOORS_OPEN:
        			break;
        		case DOORS_OPEN_NO_PENDING_CALLS:
        		     warning("R-T.7 Violated: Car doors open at Floor " + currentFloor+ " "+ msg.getHallway() + " when there is no pending calls");
        			break;
        		}
        	}
        	state = newState;
        	
        }
	}
	
	private class RT10StateMachine{
		RT10States state[][] = new RT10States[2][2];
		boolean warningIssued = false;

		public RT10StateMachine(){
			state[Hallway.BACK.ordinal()][Side.LEFT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.FRONT.ordinal()][Side.LEFT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.BACK.ordinal()][Side.RIGHT.ordinal()] = RT10States.DOORS_STOPPED;
			state[Hallway.FRONT.ordinal()][Side.RIGHT.ordinal()] = RT10States.DOORS_STOPPED;
		}
		
		public void receive(ReadableDoorMotorPayload msg){
			updateState(msg);
		}
		
		private void updateState(ReadableDoorMotorPayload msg){
			RT10States previousState = state[msg.getHallway().ordinal()][msg.getSide().ordinal()];
			RT10States newState = previousState;
			
			if(msg.command() == DoorCommand.NUDGE && !reversal[msg.getHallway().ordinal()][msg.getSide().ordinal()]){
				newState = RT10States.DOORS_NUDGING_NO_REVERSAL;
			}
			else if (msg.command() == DoorCommand.NUDGE && reversal[msg.getHallway().ordinal()][msg.getSide().ordinal()]){
				newState = RT10States.DOOR_NUDGING_AFTER_REVERSAL;
			}
			else if (msg.command() == DoorCommand.STOP){
				newState = RT10States.DOORS_STOPPED;
			}
			
			if(newState != previousState){
				switch(newState){
				case DOORS_NUDGING_NO_REVERSAL:
					if (!warningIssued){
						warning("R-T.10 Violated: at Floor " + currentFloor+ " "+msg.getHallway()+" " + msg.getSide()+" Car doors are nudging when no door reversals have occured");
						warningIssued = true;
					}
				case DOOR_NUDGING_AFTER_REVERSAL:
					break;
				case DOORS_STOPPED:
					warningIssued = false;
					break;
				}
			}
			state[msg.getHallway().ordinal()][msg.getSide().ordinal()] = newState;
		}
	}
}
