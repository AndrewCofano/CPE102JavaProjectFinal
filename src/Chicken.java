import processing.core.PImage;
import java.util.List;

public class Chicken
    extends MobileAnimatedActor
{
    public boolean isCharizard;

    public Chicken(String name, Point position, int rate, int animation_rate,
                   List<PImage> imgs)
    {
        super(name, position, rate, animation_rate, imgs);
        isCharizard = false;
    }

    public boolean isMiner()
    {
        return false;
    }


    protected boolean canPassThrough(WorldModel world, Point pt)
    {
        return !world.isOccupied(pt);
    }


    public void move(WorldModel world)
    {
        world.moveEntity(this, nextPosition(world, null));
    }

    public Action createAction(WorldModel world, ImageStore imageStore)
    {
        Action[] action = { null };
        action[0] = ticks -> {
            removePendingAction(action[0]);

            move(world);

            scheduleAction(world, this, this.createAction(world, imageStore),
                    ticks+this.getRate());
        };
        return action[0];
    }

    public boolean valid_NextPosition(WorldModel world, Point pt)
    {
        if(!world.isOccupied(pt) && world.withinBounds(pt))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
