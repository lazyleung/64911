/*
ECE649 FALL 2014
Group 11
Jonathan Leung/jkleung1
Eric Newhall/enewhall
Mengzhe Li/mzli 
Ting Xu/tingx
*/

package simulator.elevatorcontrol;

import java.util.BitSet;
import simulator.framework.Direction;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {

    public DriveSpeedCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 5, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    public DriveSpeedCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 5, MessageDictionary.DRIVE_SPEED_CAN_ID);
    }

    //this method is required for reflection
    public void set(double s, Direction d) {
        setSpeed(s);
        setDirection(d);
    }

    // Get & Set for Speed    
    public void setSpeed(double s) {
        BitSet b = getMessagePayload();
        addUnsignedIntToBitset(b, (int)(s * 1000.0), 0, 32);
        setMessagePayload(b, getByteSize());
    }

    public double getSpeed() {
        return getUnsignedIntFromBitset(getMessagePayload(), 0, 32)/1000.0;
    }

    // Get & Set for Direction
    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dir.ordinal(), 32, 8);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getIntFromBitset(getMessagePayload(), 32, 8);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }

    @Override 
    public String payloadToString() {
        return "Speed=" + getSpeed() + " Direction=" + getDirection();
    }
}