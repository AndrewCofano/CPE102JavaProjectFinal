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

   private static <T> void setCell(T[][] grid, Point pt, T v)
   {
      grid[pt.y][pt.x] = v;
   }
/*
   public static int heurisitc_cost_estimate(Node start, Node goal)
   {
      int start_x = start.getPosition().x;
      int start_y = start.getPosition().y;
      int goal_x = goal.getPosition().x;
      int goal_y = goal.getPosition().y;

      int distance = Math.abs(goal_x - start_x) + Math.abs(goal_y - start_y);
      return distance;
   }

    public boolean valid_neighbor(Node neighbor)
    {
       Point pt = neighbor.getPosition();
       if (!withinBounds(pt))
       {
          return false;
       }
        WorldObject class_check = getCell(occupancy, pt);
        if(!(class_check instanceof  Miner || class_check instanceof Obstacle ||
                class_check instanceof OreBlob || class_check instanceof Vein))
        {
            return true;
        }
        if(class_check instanceof Blacksmith)
        {
            return false;
        }
       else
       {
            return false;
       }
    }

   public ArrayList<Node> neighbor_nodes(Node current)
   {
       ArrayList<Node> possible_neighbors = new ArrayList<>();

       Point up_neighbor_pt = new Point(current.getPosition().x, current.getPosition().y-1);
       Node up_neighbor = new Node(up_neighbor_pt);
       up_neighbor.setG_Score(current.getG_Score() + 1);
       possible_neighbors.add(up_neighbor);

       Point right_neighbor_pt = new Point(current.getPosition().x+1, current.getPosition().y);
       Node right_neighbor = new Node(right_neighbor_pt);
       right_neighbor.setG_Score(current.getG_Score() + 1);
       possible_neighbors.add(right_neighbor);

       Point down_neighbor_pt = new Point(current.getPosition().x, current.getPosition().y+1);
       Node down_neighbor = new Node(down_neighbor_pt);
       down_neighbor.setG_Score(current.getG_Score()+1);
       possible_neighbors.add(down_neighbor);

       Point left_neighbor_pt = new Point(current.getPosition().x-1, current.getPosition().y);
       Node left_neighbor = new Node(left_neighbor_pt);
       left_neighbor.setG_Score(current.getG_Score() + 1);
       possible_neighbors.add(left_neighbor);

       ArrayList<Node> valid_neighbors = new ArrayList<>();
       for (Node neighbor: possible_neighbors)
       {
           if (valid_neighbor(neighbor))
           {
               valid_neighbors.add(neighbor);
           }
       }

      return valid_neighbors;
   }


   public LinkedList<Node> A_Star(Node start, Node goal)
   {
      ArrayList<Node> closedSet = new ArrayList<>();
      OrderedList<Node> openSet =  new OrderedList<>();
      start.setF_Score(start, goal);
      openSet.insert(start, start.getF_Score());
      Map<Node, Node> came_from = new HashMap<>();
      //Node[][] came_from = new Node[30][40];

      while (openSet.size() != 0)
      {
         Node current = openSet.head().item;
         if (current.getPosition().x == goal.getPosition().x && current.getPosition().y == goal.getPosition().y)
         {
            return reconstruct_path(came_from, goal);
         }

         openSet.remove(current);
         closedSet.add(current);
         ArrayList<Node> neighbor_list = neighbor_nodes(current);

         for (Node neighbor: neighbor_list)
         {
            if (closedSet.contains(neighbor))
            {
               continue;
            }

            int tentative_g_score = current.getG_Score() + heurisitc_cost_estimate(current, neighbor);

            if (!(openSet.contains(neighbor)) || (tentative_g_score < neighbor.getG_Score()))
            {
               came_from.put(neighbor, current);
               //System.out.println(neighbor.getPosition() + " came from " + current.getPosition());
               neighbor.setG_Score(tentative_g_score);
               neighbor.setF_Score(neighbor, goal);

               if (!(openSet.contains(neighbor)))
               {
                  openSet.insert(neighbor, neighbor.getF_Score());
               }
            }
         }
      }
      //System.out.println("FAILING");
      LinkedList<Node> failure = null;
      return failure;
   }

   public LinkedList<Node> reconstruct_path(Map<Node, Node> came_from, Node current)
   {
       LinkedList<Node> total_path = new LinkedList<>();
       total_path.add(current);
      //System.out.println("\t\t\t " + came_from.containsKey(current) + " " + current.getPosition());
       while (came_from.containsKey(current))
       {
           current = came_from.get(current);
           //System.out.println("yay");
           //System.out.println(current.getPosition().y);
           total_path.add(0, current);
       }
       return total_path;
   }
   */
}
