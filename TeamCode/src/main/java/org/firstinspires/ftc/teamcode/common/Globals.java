package org.firstinspires.ftc.teamcode.common;

public class Globals {

    /**
     * Robot State Constants
     */
    public static boolean IS_SCORING = false;
    public static boolean IS_INTAKING = false;


    public static void startIntaking() {
        IS_SCORING = false;
        IS_INTAKING = true;
    }

    public static void stopIntaking() {
        IS_SCORING = false;
        IS_INTAKING = false;
    }

    public static int getTargetIndex() {
        int index = 0;

        System.out.println("CURRENT INDEX");
        System.out.println(index);
//        System.out.println(Range.clip(index, 0, 11));

//        return Range.clip(index, 0, 5);
        return index;
    }
}