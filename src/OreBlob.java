import processing.core.PImage;
import java.util.List;

public class OreBlob
   extends MobileAnimatedActor
{
   private static final int QUAKE_ANIMATION_RATE = 100;

   public OreBlob(String name, Point position, int rate, int animation_rate,
      List<PImage> imgs)
   {
      super(name, position, rate, animation_rate, imgs);

   }

   protected boolean canPassThrough(WorldModel world, Point pt)
   {
      return !world.isOccupied(pt) || world.getTileOccupant(pt) instanceof Ore;
   }

   private boolean move(WorldModel world, WorldEntity target)
   {
      if (target == null)
      {
         return false;
      }

      if (adjacent(getPosition(), target.getPosition()))
      {
         target.remove(world);
         return true;
      }
      else
      {
         Point new_pt = nextPosition(world, target.getPosition());
         WorldEntity old_entity = world.getTileOccupant(new_pt);
         if (old_entity != null && old_entity != this)
         {
            old_entity.remove(world);
         }
         world.moveEntity(this, new_pt);
         return false;
      }
   }

   public Action createAction(WorldModel world, ImageStore imageStore)
   {
      Action[] action = { null };
      action[0] = ticks -> {
         removePendingAction(action[0]);

         WorldEntity target = world.findNearest(getPosition(), Vein.class);
         long nextTime = ticks + getRate();

         if (target != null)
         {
            Point tPt = target.getPosition();

            if (move(world, target))
            {
               Quake quake = createQuake(world, tPt, ticks, imageStore);
               world.addEntity(quake);
               nextTime = nextTime + getRate();
            }
         }

         scheduleAction(world, this, createAction(world, imageStore),
            nextTime);
         
      };
      return action[0];
   }

   protected boolean isMiner()
   {
      return false;
   }

   private Quake createQuake(WorldModel world, Point pt, long ticks,
      ImageStore imageStore)
   {
      Quake quake = new Quake("quake", pt, QUAKE_ANIMATION_RATE,
         imageStore.get("quake"));
      quake.schedule(world, ticks, imageStore);
      return quake;
   }
}
