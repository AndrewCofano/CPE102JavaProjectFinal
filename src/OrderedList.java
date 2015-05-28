import sun.plugin.javascript.navig4.Link;

import java.util.List;
import java.util.LinkedList;

public class OrderedList<T>
{
   private List<ListItem<T>> list;

   public OrderedList()
   {
      list = new LinkedList<>();
   }

   public LinkedList<T> getItems()
   {
      LinkedList<T> new_list = new LinkedList<>();
      for (ListItem<T> item : list)
      {
         new_list.add(item.item);
      }
      return new_list;
   }

   public void insert(T item, long ord)
   {
      int idx = 0;
      for (ListItem<T> lItem : list)
      {
         if (lItem.ord >= ord)
         {
            break;
         }
         idx++;
      }

      list.add(idx, new ListItem<>(item, ord));
   }


   public boolean contains(T item)
   {
       for (ListItem<T> listItem: list)
       {
           if (listItem.item.equals(item))
           {
               return true;
           }
       }
       return false;
   }

   public void remove(T item)
   {
      int idx = 0;
      for (ListItem<T> lItem : list)
      {
         if (lItem.item.equals(item))
         {
            break;
         }
         idx++;
      }

      if (idx < list.size())
      {
         list.remove(idx);
      }
   }

   public ListItem<T> head()
   {
      if (!list.isEmpty())
      {
         return list.get(0);
      }
      else
      {
         return null;
      }
   }

   public void pop()
   {
      if (!list.isEmpty())
      {
         list.remove(0);
      }
   }

   public int size()
   {
      return list.size();
   }

   public static class ListItem<T>
   {
      public final T item;
      public final long ord;

      public ListItem(T item, long ord)
      {
         this.item = item;
         this.ord = ord;
      }
   }
}
