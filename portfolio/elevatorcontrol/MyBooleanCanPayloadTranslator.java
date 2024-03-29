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
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

/**
 * Translates a single boolean value into a 1-byte payload.
 * 
 * @author Jonathan Leung
 */
public class MyBooleanCanPayloadTranslator extends CanPayloadTranslator {

    
    /**
     * Constructor for use with WriteableCanMailbox objects
     * @param payload
     */
    public MyBooleanCanPayloadTranslator(WriteableCanMailbox payload) {
        super(payload, 1);
    }

    /**
     * Constructor for use with ReadableCanMailbox objects
     * @param payload
     */

    public MyBooleanCanPayloadTranslator(ReadableCanMailbox payload) {
        super(payload, 1);
    }
    
    //required for reflection
    public void set(boolean value) {
        setValue(value);
    }

    
    public void setValue(boolean value) {       
        BitSet b = new BitSet(1);
        b.set(0, value);
        setMessagePayload(b, getByteSize());
    }
    
    public boolean getValue() {
        return getMessagePayload().get(0);
    }
    
    @Override
    public String payloadToString() {
        return Boolean.toString(getValue());
    }
}
