import processing.core.PApplet;
import processing.core.PImage;

public class WorldView
{
   private PApplet screen;
   private WorldModel world;
   private int tileWidth;
   private int tileHeight;
   private Viewport viewport;

   public WorldView(int numCols, int numRows, PApplet screen, WorldModel world,
      int tileWidth, int tileHeight)
   {
      this.screen = screen;
      this.world = world;
      this.tileWidth = tileWidth;
      this.tileHeight = tileHeight;
      this.viewport = new Viewport(numRows, numCols);
   }

   public void drawViewport()
   {
      drawBackground();
      drawEntities();
   }

   private void drawBackground()
   {
      for (int row = 0; row < viewport.getNumRows(); row++)
      {
         for (int col = 0; col < viewport.getNumCols(); col++)
         {
            Point wPt = viewportToWorld(viewport, col, row);
            PImage img = world.getBackground(wPt).getImage();
            screen.image(img, col * tileWidth, row * tileHeight);
         }
      }
   }

   private void drawEntities()
   {
      for (WorldEntity entity : world.getEntities())
      {
         Point pt = entity.getPosition();
         if (viewport.contains(pt))
         {
            Point vPt = worldToViewport(viewport, pt.x, pt.y);
            screen.image(entity.getImage(), vPt.x * tileWidth,
               vPt.y * tileHeight);
         }
      }
   }

   public void updateView(int dx, int dy)
   {
      int new_x = clamp(viewport.getCol() + dx, 0,
         world.getNumCols() - viewport.getNumCols());
      int new_y = clamp(viewport.getRow() + dy, 0,
         world.getNumRows() - viewport.getNumRows());
      viewport.shift(new_y, new_x);
   }

   private static int clamp(int v, int min, int max)
   {
      return Math.min(max, Math.max(v, min));
   }

   private static Point viewportToWorld(Viewport viewport, int col, int row)
   {
      return new Point(col + viewport.getCol(), row + viewport.getRow());
   }

   private static Point worldToViewport(Viewport viewport, int col, int row)
   {
      return new Point(col - viewport.getCol(), row - viewport.getRow());
   }
}
