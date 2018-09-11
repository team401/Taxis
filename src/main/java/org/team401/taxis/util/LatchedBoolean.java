//File originally from FRC Team 254's 2018 Robot code

package org.team401.taxis.util;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class LatchedBoolean {
    private boolean mLast = false;

    public boolean update(boolean newValue) {
        boolean ret = false;
        if (newValue && !mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}