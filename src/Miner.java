import processing.core.PImage;
import java.util.List;

public abstract class Miner
   extends MobileAnimatedActor
{
   private int resource_limit;
   private int resource_count;
   protected Class<?> seeking;

   public Miner(String name, Point position, int rate, int animation_rate,
      int resource_limit, int resource_count, Class<?> seeking,
      List<PImage> imgs)
   {
      super(name, position, rate, animation_rate, imgs);
      this.resource_limit = resource_limit;
      this.resource_count = resource_count;
      this.seeking = seeking;
   }

   protected boolean isMiner()
   {
      return true;
   }

   protected boolean isCharizard()
   {
      return false;
   }

   public void setResourceCount(int count)
   {
      this.resource_count = count;
   }

   public int getResourceCount()
   {
      return this.resource_count;
   }

   public int getResourceLimit()
   {
      return this.resource_limit;
   }

   protected boolean canPassThrough(WorldModel world, Point pt)
   {
      return !world.isOccupied(pt);
   }

   protected abstract Miner transform(WorldModel world);
   protected abstract boolean move(WorldModel world, WorldEntity ore);

   public Action createAction(WorldModel world, ImageStore imageStore)
   {
      Action[] action = { null };
      action[0] = ticks -> {
         removePendingAction(action[0]);

         WorldEntity target = world.findNearest(getPosition(), seeking);

         Actor newEntity = this;
         if (move(world, target))
         {
            newEntity = tryTransform(world);
         }

         scheduleAction(world, newEntity,
            newEntity.createAction(world, imageStore),
            ticks + newEntity.getRate());
      };
      return action[0];
   }

   /*public MachoMan transformToMachoMan(WorldModel world)
   {
      return new MachoMan(getName(),getPosition(),getRate(),
              getAnimationRate(),Chicken.class,ImageStore.macho_imgs);
   }*/

   private Miner tryTransform(WorldModel world)
   {
      Miner newEntity = transform(world);
      if (this != newEntity)
      {
         this.remove(world);
         world.addEntity(newEntity);
         newEntity.scheduleAnimation(world);
      }
      return newEntity;
   }
}
