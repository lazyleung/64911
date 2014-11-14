
//ECE649 FALL 2014
//Group 11
//Jonathan Leung/jkleung1
//Eric Newhall/enewhall
//Mengzhe Li/mzli
//Ting Xu/tingx

package simulator.elevatorcontrol;

import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.BooleanCanPayloadTranslator;

public class HallCallCanPayloadTranslator extends BooleanCanPayloadTranslator {
	/**
     * CAN translator for messages from atfloor sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
    public HallCallCanPayloadTranslator(WriteableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload);
        }

    /**
     * CAN translator for messages from atfloor sensors
     * @param payload CAN payload object whose message is interpreted by this translator
     * @param floor replication index
     * @param hallway replication index
     */
    public HallCallCanPayloadTranslator(ReadableCanMailbox payload, int floor, Hallway hallway, Direction direction) {
        super(payload);
        }

}
