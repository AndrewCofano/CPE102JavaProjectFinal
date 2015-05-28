import processing.core.PImage;
import java.util.List;
import java.util.Iterator;

public class WorldObject
{
   private String name;
   private List<PImage> imgs;
   private Iterator<PImage> iter;
   private PImage currentImg;

   public WorldObject(String name, List<PImage> imgs)
   {
      this.name = name;
      this.imgs = imgs;
      this.iter = imgs.iterator();
      nextImage();
   }

   public String getName()
   {
      return this.name;
   }

   public String toString()
   {
      return "unknown";
   }

   public PImage getImage()
   {
      return currentImg;
   }

   public void nextImage()
   {
      if (iter.hasNext())
      {
         this.currentImg = iter.next();
      }
      else
      {
         this.iter = this.imgs.iterator();
         this.currentImg = iter.next();
      }
   }

   protected List<PImage> getImages()
   {
      return imgs;
   }
}
