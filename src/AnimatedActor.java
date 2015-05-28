import processing.core.PImage;
import java.util.List;

public abstract class AnimatedActor
   extends Actor
{
   private int animation_rate;

   public AnimatedActor(String name, Point position, int rate,
      int animation_rate, List<PImage> imgs)
   {
      super(name, position, rate, imgs);
      this.animation_rate = animation_rate;
   }

   public int getAnimationRate()
   {
      return this.animation_rate;
   }

   public void schedule(WorldModel world, long time, ImageStore imageStore)
   {
      super.schedule(world, time, imageStore);
      scheduleAnimation(world);
   }

   protected void scheduleAnimation(WorldModel world)
   {
      Actor.scheduleAction(world, this, createAnimationAction(world, 0),
         animation_rate);
   }

   protected Action createAnimationAction(WorldModel world, int repeatCount)
   {
      Action action[] = { null };
      action[0] = currentTime -> {
            this.removePendingAction(action[0]);
            this.nextImage();

            if (repeatCount != 1)
            {
               Actor.scheduleAction(world, this,
                  createAnimationAction(world, Math.max(repeatCount - 1, 0)),
                  currentTime + animation_rate);
            }
         };
      return action[0];
   }
}
