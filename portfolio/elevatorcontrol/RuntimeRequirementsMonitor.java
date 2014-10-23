package simulator.elevatorcontrol;

import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;

public class RuntimeRequirementsMonitor extends RuntimeMonitor  {
	
	RT6StateMachine RT6State = new RT6StateMachine();
	RT7StateMachine RT7State = new RT7StateMachine();

	String RT6Warnings, RT7Warnings;
	

	@Override
	public void timerExpired(Object callbackData) {
		//do nothing
		
	}

	@Override
	protected String[] summarize() {
		 String[] arr = new String[3];
	        arr[0] = RT6Warnings;
	        arr[1] = RT7Warnings;
	        return arr;
	}

	  public void receive(ReadableDoorClosedPayload msg) {
		  RT7State.receive(msg);
	    }
	
	
	
	
	
	private static enum RT6States{
		MOVING,
		STOPPED,
		STOPPED_NO_PENDING_CALLS
	}
	
	private static enum RT7States{
		DOORS_CLOSED,
		DOORS_OPENED,
		DOORS_OPENED_NO_PENDING_CALLS
	}
	
	
	private class RT6StateMachine{
		RT6States state;
		
		public RT6StateMachine(){
			state = RT6States.STOPPED;
		}		
		
	}
	
	private class RT7StateMachine{
		RT7States state;
		
		public RT7StateMachine(){
			state = RT7States.DOORS_CLOSED;
		}
		
        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }
        
        private void updateState(Hallway h){
        	RT7States previousState = state;
        	RT7States newState = previousState;
        	
        }
	}
}
