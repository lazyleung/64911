//ECE649 FALL 2014
//Group 11
//Jonathan Leung/jkleung1
//Eric Newhall/enewhall
//Mengzhe Li/mzli
//Ting Xu/tingx

package simulator.elevatorcontrol;

import simulator.elevatorcontrol.BooleanCanTranslator;
import simulator.elevatorcontrol.MessageDictionary;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class CarCallCanPayloadTranslator extends BooleanCanTranslator {

	/**
     * Constructor for WriteableCanMailbox.  You should always implement both a
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public CarCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

    /**
     * Constructor for ReadableCanMailbox.  You should always implement both a
     * Writeable and Readable constructor so the same translator can be used for
     * both objects
     * @param payload
     */
    public CarCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway) {
    	super(payload, MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway), "CarCall" + ReplicationComputer.makeReplicationString(floor, hallway));
    }

}
