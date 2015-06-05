import processing.core.PImage;
import java.util.List;

public class MachoMan
    extends MobileAnimatedActor
{
    private Class<?> seeking;
    private int chicken_count;
    private List<PImage> imgs;
    private ImageStore imageStore;

    public MachoMan(String name, Point position, int rate, int animation_rate,
                    Class<?> seeking, List<PImage> imgs, ImageStore imageStore)
    {
        super(name, position, rate, animation_rate, imgs);
        this.seeking = seeking;
        this.chicken_count = 0;
        this.imageStore = imageStore;

    }

    public void setSeeking(Class<?> new_seeking){
        this.seeking = new_seeking;
    }

    public boolean isMiner()
    {
        return false;
    }

    public boolean canPassThrough(WorldModel world, Point pt)
    {
        return !world.isOccupied(pt);
    }

    public void move(WorldModel world, WorldEntity target, long ticks)
    {
        /*
        if (chicken == null)
        {
            return false;
            break;
        }
        */
        if (target != null) {
            if (adjacent(getPosition(), target.getPosition())) {
                target.remove(world);
                this.chicken_count++;
                //return true;
                if (chicken_count == 1) {
                    List<PImage> new_imgs = imageStore.get("char");
                    this.setImages(new_imgs);
                    this.setAnimation_rate(80);
                    this.setRate(500);
                }
                else if (chicken_count == 2)
                {
                    List<PImage> new_imgs = imageStore.get("char2");
                    this.setImages(new_imgs);
                    this.setAnimation_rate(80);
                    this.setRate(400);
                }
                else if (chicken_count >= 3)
                {

                    List<PImage> new_imgs = imageStore.get("char3");
                    this.setImages(new_imgs);
                    this.setAnimation_rate(80);
                    this.setRate(200);
                }

            }
            else {
                world.moveEntity(this, nextPosition(world, target.getPosition()));
                //return false;

            }
        }
        else{
            world.moveEntity(this, getPosition());
        }
    }

    public Action createAction(WorldModel world, ImageStore imageStore)
    {
        Action[] action = { null };
        action[0] = ticks -> {
            removePendingAction(action[0]);

            WorldEntity target = world.findNearest(getPosition(), seeking);
            Actor newEntity = this;
            move(world, target, ticks);

            scheduleAction(world, newEntity,
                    newEntity.createAction(world, imageStore),
                    ticks + newEntity.getRate());
        };
        return action[0];
    }



}
