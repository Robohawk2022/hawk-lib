package robohawks;

public class Util {

    public static void log(String text, Object... args) {
        System.out.printf(text, args);
        if (!text.endsWith("%n")) {
            System.out.println();
        }
    }

}
