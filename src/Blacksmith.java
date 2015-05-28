import processing.core.PImage;
import java.util.List;

public class Blacksmith
   extends WorldEntity
{
   public Blacksmith(String name, Point position, List<PImage> imgs)
   {
      super(name, position, imgs);
   }

   public String toString()
   {
      return String.format("blacksmith %s %d %d", this.getName(),
         this.getPosition().x, this.getPosition().y);
   }
}
