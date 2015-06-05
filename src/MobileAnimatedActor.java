import processing.core.PImage;
import java.util.*;
import java.lang.Math;
//import java.util.LinkedList;
//import java.util.List;
//import static java.lang.Math.abs;
//import java.util.ArrayList;
//import java.util.HashMap;

public abstract class MobileAnimatedActor
   extends AnimatedActor {

   public ArrayList<Node> closedSet;
   public OrderedList<Node> openSet;
   public LinkedList<Node> total_path;
   public boolean isCharizard;

   public static final Random chicken_rand = new Random();
   public static final int CHICKEN_MAX_POSITION = 4;
   public static final int CHICKEN_MIN_POSITION = 1;

   public MobileAnimatedActor(String name, Point position, int rate,
                              int animation_rate, List<PImage> imgs) {

      super(name, position, rate, animation_rate, imgs);
   }

   public LinkedList<Node> getTotalPath()
   {
      return this.total_path;
   }

   public OrderedList<Node> getOpenSet()
   {
      return this.openSet;
   }

   protected abstract boolean isMiner();

   protected Point nextPosition(WorldModel world, Point dest_pt) {
      /*
      int horiz = Integer.signum(dest_pt.x - getPosition().x);
      Point new_pt = new Point(getPosition().x + horiz, getPosition().y);

      if (horiz == 0 || !canPassThrough(world, new_pt))
      {
         int vert = Integer.signum(dest_pt.y - getPosition().y);
         new_pt = new Point(getPosition().x, getPosition().y + vert);

         if (vert == 0 || !canPassThrough(world, new_pt))
         {
            new_pt = getPosition();
         }
      }
      */
      if (this instanceof Chicken)
      {
         int randint = chicken_rand.nextInt(4);
         Point next_position;
         Chicken chick = (Chicken)this;
         if (randint == 0)
         {
            next_position = new Point(chick.getPosition().x-1, chick.getPosition().y);
            if ((chick.valid_NextPosition(world, next_position)))
            {
               return next_position;
            }
            else
            {
               return chick.getPosition();
            }
         }
         if (randint == 1)
         {
            next_position = new Point(chick.getPosition().x, chick.getPosition().y-1);
            if ((chick.valid_NextPosition(world, next_position)))
            {
               return next_position;
            }
            else
            {
               return chick.getPosition();
            }
         }
         if (randint == 2)
         {
            next_position = new Point(chick.getPosition().x+1, chick.getPosition().y);
            if (chick.valid_NextPosition(world, next_position))
            {
               return next_position;
            }
            else
            {
               return chick.getPosition();
            }
         }
         if (randint == 3)
         {
            next_position = new Point(chick.getPosition().x, chick.getPosition().y+1);
            if (chick.valid_NextPosition(world, next_position))
            {
               return next_position;
            }
            else
            {
               return chick.getPosition();
            }
         }
      }
      Node current = new Node(this.getPosition());
      Node destination = new Node(dest_pt);
      LinkedList<Node> path = A_Star(world, current, destination);
      if (path != null) {
         Point new_pt = path.get(1).getPosition();
         return new_pt;
      } else {
         //System.out.println("Next position == null");
         return this.getPosition();
      }
      //Point new_pt = path.getFirst().getPosition();
      //return new_pt;
   }

   protected static boolean adjacent(Point p1, Point p2) {
      return (p1.x == p2.x && Math.abs(p1.y - p2.y) == 1) ||
              (p1.y == p2.y && Math.abs(p1.x - p2.x) == 1);
   }

   protected abstract boolean canPassThrough(WorldModel world, Point new_pt);

   public static int heurisitc_cost_estimate(Node start, Node goal) {
      int start_x = start.getPosition().x;
      int start_y = start.getPosition().y;
      int goal_x = goal.getPosition().x;
      int goal_y = goal.getPosition().y;

      int distance = Math.abs(goal_x - start_x) + Math.abs(goal_y - start_y);
      return distance;
   }



   public boolean valid_neighbor(WorldModel world, Node neighbor) {
      Point pt = neighbor.getPosition();
      if (!world.withinBounds(pt)) {
         return false;
      }
      WorldObject class_check = WorldModel.getCell(world.getOccupancy(), pt);
      if (this.isMiner()) {
         if (!(class_check instanceof Miner || class_check instanceof Obstacle ||
                 class_check instanceof OreBlob || class_check instanceof Vein ||
                 class_check instanceof MachoMan || class_check instanceof Chicken)) {
            return true;
         }
         else if (!(class_check instanceof Blacksmith)) {
            return false;
         } else {
            return false;
         }
      } else if (!this.isMiner()) {
         if (!(class_check instanceof Miner || class_check instanceof Obstacle ||
                 class_check instanceof OreBlob || class_check instanceof Blacksmith)) {
            return true;
         } else
         {
            return false;
         }
      }
      /*
      }
      {
         return true;
      }
      if(class_check instanceof Blacksmith)
      {
         return false;
      }
      else */
      {
         return false;
      }

   }


   public ArrayList<Node> neighbor_nodes(WorldModel world, Node current)
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
         if (valid_neighbor(world, neighbor))
         {
            valid_neighbors.add(neighbor);
         }
      }

      return valid_neighbors;
   }


   public LinkedList<Node> A_Star(WorldModel world, Node start, Node goal)
   {
      closedSet = new ArrayList<>();
      openSet =  new OrderedList<>();
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
         ArrayList<Node> neighbor_list = neighbor_nodes(world, current);

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
      total_path = new LinkedList<>();
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
}
