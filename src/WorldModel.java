import processing.core.PImage;
import java.util.*;
import java.util.concurrent.SynchronousQueue;

public class WorldModel
{
   private Background[][] background;
   private WorldEntity[][] occupancy;
   private List<WorldEntity> entities;
   private static int numRows;
   private static int numCols;
   private OrderedList<Action> actionQueue;

   private static final String MACHO_IMG_KEY = "macho";
   private static final String MACHO_MAN_NAME = "Aaron";
   private static final int MACHO_MAN_RATE = 601;
   private static final int MACHO_MAN_ANIMATE = 300;

   private static final String CHICKEN_IMG_KEY = "chicken";
   private static final int CHICKEN_RATE = 420;
   private static final int CHICKEN_ANIMATE = 100;


   public WorldModel(int numRows, int numCols, Background background)
   {
      this.background = new Background[numRows][numCols];
      this.occupancy = new WorldEntity[numRows][numCols];
      this.numRows = numRows;
      this.numCols = numCols;
      this.entities = new LinkedList<>();
      this.actionQueue = new OrderedList<>();

      for (int row = 0; row < numRows; row++)
      {
         Arrays.fill(this.background[row], background);
      }
   }

   public WorldEntity[][] getOccupancy()
   {
      return this.occupancy;
   }

   public boolean withinBounds(Point pt)
   {
      return pt.x >= 0 && pt.x < numCols && pt.y >= 0 && pt.y < numRows;
   }

   public int getNumRows()
   {
      return numRows;
   }

   public int getNumCols()
   {
      return numCols;
   }

   public List<WorldEntity> getEntities()
   {
      return entities;
   }

   public boolean isOccupied(Point pt)
   {
      return withinBounds(pt) && getCell(occupancy, pt) != null;
   }

   public WorldEntity findNearest(Point pt, Class type)
   {
      List<WorldEntity> ofType = new LinkedList<>();
      for (WorldEntity entity : entities)
      {
         if (type.isInstance(entity))
         {
            ofType.add(entity);
         }
      }

      return nearestEntity(ofType, pt);
   }

   public void addEntity(WorldEntity entity)
   {
      Point pt = entity.getPosition();
      if (withinBounds(pt))
      {
         WorldEntity old = getCell(occupancy, pt);
         if (old != null)
         {
            old.remove(this);
         }
         setCell(occupancy, pt, entity);
         entities.add(entity);
      }
   }

   public void moveEntity(WorldEntity entity, Point pt)
   {
      if (withinBounds(pt))
      {
         Point oldPt = entity.getPosition();
         setCell(occupancy, oldPt, null);
         removeEntityAt(pt);
         setCell(occupancy, pt, entity);
         entity.setPosition(pt);
      }
   }

   public void removeEntity(WorldEntity entity)
   {
      removeEntityAt(entity.getPosition());
   }

   public void removeEntityAt(Point pt)
   {
      if (withinBounds(pt) && getCell(occupancy, pt) != null)
      {
         WorldEntity entity = getCell(occupancy, pt);
         entity.setPosition(new Point(-1, -1));
         entities.remove(entity);
         setCell(occupancy, pt, null);
      }
   }

   public Background getBackground(Point pt)
   {
      return withinBounds(pt) ? getCell(background, pt) : null;
   }

   public void setBackground(Point pt, Background bgnd)
   {
      if (withinBounds(pt))
      {
         setCell(background, pt, bgnd);
      }
   }

   public WorldEntity getTileOccupant(Point pt)
   {
      return withinBounds(pt) ? getCell(occupancy, pt) : null;
   }

   public void scheduleAction(Action action, long time)
   {
      actionQueue.insert(action, time);
   }

   public void unscheduleAction(Action action)
   {
      actionQueue.remove(action);
   }

   public void updateOnTime(long time)
   {
      OrderedList.ListItem<Action> next = actionQueue.head();
      while (next != null && next.ord < time)
      {
         actionQueue.pop();
         next.item.execute(time);
         next = actionQueue.head();
      }
   }

   private static WorldEntity nearestEntity(List<WorldEntity> entities,
      Point pt)
   {
      if (entities.size() == 0)
      {
         return null;
      }
      WorldEntity nearest = entities.get(0);
      double nearest_dist = distance_sq(nearest.getPosition(), pt);

      for (WorldEntity entity : entities)
      {
         double dist = distance_sq(entity.getPosition(), pt);
         if (dist < nearest_dist)
         {
            nearest = entity;
            nearest_dist = dist;
         }
      }

      return nearest;
   }

   private static double distance_sq(Point p1, Point p2)
   {
      double dx = p1.x - p2.x;
      double dy = p1.y - p2.y;
      return dx * dx + dy * dy;
   }

   public static <T> T getCell(T[][] grid, Point pt)
   {
      return grid[pt.y][pt.x];
   }

   public static <T> void setCell(T[][] grid, Point pt, T v)
   {
      grid[pt.y][pt.x] = v;
   }

   public void createMachoMan(Point pt, ImageStore imageStore, long next_time)
   {
      List<PImage> macho_imgs = imageStore.get(MACHO_IMG_KEY);
      MachoMan new_Macho = new MachoMan(MACHO_MAN_NAME, pt,
              MACHO_MAN_RATE, MACHO_MAN_ANIMATE, Chicken.class, macho_imgs, imageStore);
      MachoMan[] list = {new_Macho};
      initializeEntities(list, imageStore, next_time);
   }

   public Chicken createSingleChicken(String name, Point pt, List<PImage> imgs)
   {
      Chicken new_chicken = new Chicken(name, pt, CHICKEN_RATE, CHICKEN_ANIMATE, imgs);
      return new_chicken;
   }

   public void initializeEntities(MobileAnimatedActor[] list, ImageStore imageStore, long action_time)
   {
      for(MobileAnimatedActor entity : list)
      {
         this.addEntity(entity);
         entity.scheduleAnimation(this);
         entity.scheduleAction(this,entity,entity.createAction(this,imageStore), action_time);
      }
   }

   public void createChickens(Point pt, ImageStore imageStore, long next_time)
   {
      List<PImage> chicken_imgs = imageStore.get(CHICKEN_IMG_KEY);
      Point chicken_1_pt = new Point(pt.x-1, pt.y+5);
      Point chicken_2_pt = new Point(pt.x-3, pt.y);
      Point chicken_3_pt = new Point(pt.x+5, pt.y+1);
      Point chicken_4_pt = new Point(pt.x+9, pt.y+9);
      Point chicken_5_pt = new Point(pt.x-3, pt.y+15);
      Point chicken_6_pt = new Point(pt.x+10, pt.y+5);
      Chicken chicken_1 = createSingleChicken("Thunder", chicken_1_pt, chicken_imgs);
      Chicken chicken_2 = createSingleChicken("Chips", chicken_2_pt, chicken_imgs);
      Chicken chicken_3 = createSingleChicken("Fang", chicken_3_pt, chicken_imgs);
      Chicken chicken_4 = createSingleChicken("Hammer", chicken_4_pt, chicken_imgs);
      Chicken chicken_5 = createSingleChicken("Fist", chicken_5_pt, chicken_imgs);
      Chicken chicken_6 = createSingleChicken("Brick", chicken_6_pt, chicken_imgs);
      Chicken[] list = {chicken_1,chicken_2,chicken_3,chicken_4,chicken_5,chicken_6};
      initializeEntities(list, imageStore, next_time);

   }
   public void queueWorldEvent(ImageStore imageStore, Background alternate, Point mouse_pt, int background_area, long next_time)
   {
      int background_length = background_area/2;
      Point new_pt;
      for (int i = 0; i < background_length; i++)
      {
         for (int j = 0; j < background_length; j++)
         {
            new_pt = new Point(mouse_pt.x+i,mouse_pt.y+j);
            if (this.withinBounds(new_pt))
            {
               this.setBackground(new_pt, alternate);

               WorldEntity entity_test = this.getCell(this.occupancy, new_pt);
               if (entity_test instanceof Miner) {
                  createMachoMan(new_pt, imageStore, next_time);
                  createChickens(new_pt, imageStore, next_time);
               }
            }

         }
      }
   }
}
