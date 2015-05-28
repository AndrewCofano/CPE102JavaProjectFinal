import processing.core.PImage;

import java.util.LinkedList;
import java.util.List;

public class MinerNotFull
   extends Miner
{
   public MinerNotFull(String name, Point position, int rate,
      int animation_rate, int resource_limit, List<PImage> imgs)
   {
      super(name, position, rate, animation_rate, resource_limit,
         0, Ore.class, imgs);
   }

   public String toString()
   {
      return String.format("miner %s %d %d %d %d %d", getName(),
         getPosition().x, getPosition().y, getResourceLimit(),
         getRate(), getAnimationRate());
   }

   protected Miner transform(WorldModel world)
   {
      if (getResourceCount() < getResourceLimit())
      {
         return this;
      }
      else
      {
         //System.out.println("The miner is TRANSFORMMMINGGGG");
         return new MinerFull(getName(), getPosition(), getRate(),
            getAnimationRate(), getResourceLimit(), getImages());
      }
   }

   protected boolean move(WorldModel world, WorldEntity ore)
   {
      if (ore == null)
      {
         return false;
      }

      if (adjacent(getPosition(), ore.getPosition()))
      {
         setResourceCount(getResourceCount() + 1);
         ore.remove(world);
         return true;
      }
      else
      {
         world.moveEntity(this, nextPosition(world, ore.getPosition()));
         return false;
      }
   }
}
