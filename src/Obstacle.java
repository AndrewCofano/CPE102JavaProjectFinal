import processing.core.PImage;
import java.util.List;

public class Obstacle
   extends WorldEntity
{
   public Obstacle(String name, Point position, List<PImage> imgs)
   {
      super(name, position, imgs);
   }

   public String toString()
   {
      return String.format("obstacle %s %d %d", this.getName(),
         this.getPosition().x, this.getPosition().y);
   }
}
