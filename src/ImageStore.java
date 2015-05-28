import java.util.Map;
import java.util.HashMap;
import java.util.List;
import java.util.LinkedList;
import processing.core.PImage;
import processing.core.PApplet;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public final class ImageStore
{
   private static final int KEYED_IMAGE_MIN = 5;

   private Map<String, List<PImage>> images;
   private List<PImage> defaultImages;

   public ImageStore(PImage defaultImage)
   {
      this.images = new HashMap<>();
      defaultImages = new LinkedList<>();
      defaultImages.add(defaultImage);
   }

   public List<PImage> get(String key)
   {
      return images.getOrDefault(key, defaultImages);
   }

   public static void loadImages(Scanner in, ImageStore imageStore,
      int tile_width, int tile_height, PApplet sketch)
      throws FileNotFoundException
   {
      while (in.hasNextLine())
      {
         processImageLine(imageStore.images, in.nextLine(), sketch);
      }
   }

   private static void processImageLine(Map<String, List<PImage>> images,
      String line, PApplet sketch)
   {
      String[] attrs = line.split("\\s");
      if (attrs.length >= 2)
      {
         String key = attrs[0];
         PImage img = sketch.loadImage(attrs[1]);
         if (img != null && img.width != -1)
         {
            List<PImage> imgs = getImages(images, key);
            imgs.add(img);

            if (attrs.length >= KEYED_IMAGE_MIN)
            {
               int r = Integer.parseInt(attrs[2]);
               int g = Integer.parseInt(attrs[3]);
               int b = Integer.parseInt(attrs[4]);
               setAlpha(img, sketch.color(r, g, b), 0);
            }
         }
      }
   }

   private static List<PImage> getImages(Map<String, List<PImage>> images,
      String key)
   {
      List<PImage> imgs = images.get(key);
      if (imgs == null)
      {
         imgs = new LinkedList<>();
         images.put(key, imgs);
      }
      return imgs;
   }

   private static final int COLOR_MASK = 0xffffff;

   // Called with color for which alpha should be set and alpha value.
   //PImage img = setAlpha(loadImage("wyvern1.bmp"), color(255, 255, 255), 0));
   private static PImage setAlpha(PImage img, int maskColor, int alpha)
   {
      int alphaValue = alpha << 24;
      int nonAlpha = maskColor & COLOR_MASK;
      img.format = PApplet.ARGB;
      img.loadPixels();
      for (int i = 0; i < img.pixels.length; i++)
      {
         if ((img.pixels[i] & COLOR_MASK) == nonAlpha)
         {
            img.pixels[i] = alphaValue | nonAlpha;
         }
      }
      img.updatePixels();
      return img;
   }
}
