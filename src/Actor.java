import processing.core.PImage;
import java.util.List;
import java.util.LinkedList;

public abstract class Actor
   extends WorldEntity
{
   private int rate;
   private List<Action> pendingActions;

   public Actor(String name, Point position, int rate, List<PImage> imgs)
   {
      super(name, position, imgs);
      this.rate = rate;
      this.pendingActions = new LinkedList<>();
   }

   public int getRate()
   {
      return this.rate;
   }

   protected void removePendingAction(Action action)
   {
      pendingActions.remove(action);
   }

   protected void addPendingAction(Action action)
   {
      pendingActions.add(action);
   }

   protected void clearPendingActions()
   {
      pendingActions.clear();
   }

   public void remove(WorldModel world)
   {
      for (Action action : pendingActions)
      {
         world.unscheduleAction(action);
      }
      clearPendingActions();
      super.remove(world);
   }

   protected static void scheduleAction(WorldModel world, Actor entity,
      Action action, long time)
   {
      entity.addPendingAction(action);
      world.scheduleAction(action, time);
   }

   public void schedule(WorldModel world, long time, ImageStore imageStore)
   {
      scheduleAction(world, this, createAction(world, imageStore),
         time + getRate());
   }

   protected abstract Action createAction(WorldModel world,
      ImageStore imageStore);
}
