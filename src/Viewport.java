public class Viewport
{
   private int row;
   private int col;
   private int numRows;
   private int numCols;

   public Viewport(int numRows, int numCols)
   {
      this.row = 0;
      this.col = 0;
      this.numRows = numRows;
      this.numCols = numCols;
   }

   public int getRow()
   {
      return row;
   }

   public int getCol()
   {
      return col;
   }

   public int getNumRows()
   {
      return numRows;
   }

   public int getNumCols()
   {
      return numCols;
   }

   public void shift(int newRow, int newCol)
   {
      this.row = newRow;
      this.col = newCol;
   }

   public boolean contains(Point p)
   {
      return p.y >= row && p.y < row + numRows &&
         p.x >= col && p.x < col + numCols;
   }
}
