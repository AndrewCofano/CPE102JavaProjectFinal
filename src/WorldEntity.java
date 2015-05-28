import processing.core.PImage;
import java.util.List;

public class WorldEntity
   extends WorldObject
{
   private Point position;

   public WorldEntity(String name, Point position, List<PImage> imgs)
   {
      super(name, imgs);
      this.position = position;
   }

   public Point getPosition()
   {
      return this.position;
   }

   public void setPosition(Point pt)
   {
      this.position = pt;
   }

   public void remove(WorldModel world)
   {
      world.removeEntity(this);
   }

}
