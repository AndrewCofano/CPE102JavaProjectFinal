import processing.core.PImage;
import java.util.Map;
import java.util.Scanner;

public final class WorldLoad
{
   private static final int PROPERTY_KEY = 0;

   private static final String BGND_KEY = "background";
   private static final int BGND_NUM_PROPERTIES = 4;
   private static final int BGND_NAME = 1;
   private static final int BGND_COL = 2;
   private static final int BGND_ROW = 3;

   private WorldLoad() {}

   public static void load(Scanner in, WorldModel world,
      ImageStore imageStore, Map<String, PropertyParser> parsers)
   {
      while (in.hasNextLine())
      {
         processLine(in.nextLine(), world, imageStore, parsers);
      }
   }

   private static void processLine(String line, WorldModel world,
      ImageStore imageStore, Map<String, PropertyParser> parsers)
   {
      String[] properties = line.split("\\s");
      if (properties.length > 0)
      {
         PropertyParser parser = parsers.get(properties[PROPERTY_KEY]);
         if (parser != null)
         {
            parser.parse(properties);
         }
      }
   }
}
