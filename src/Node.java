public class Node
{

    private Point position;
    private int g_score;
    private int h_estimate;
    private int f_score;

    public Node(Point pt)
    {
        this.position = pt;
    }

    public boolean equals(Object o)
    {
        Node other = (Node)o;
        return this.position.x == other.getPosition().x && this.position.y == other.getPosition().y;/* &&
                this.g_score == other.getG_Score() && this.h_estimate == other.getH_Score() &&
                this.f_score == other.getF_Score();*/
    }

    public int hashCode()
    {
        return 1;
    }

    public Point getPosition()
    {
        return this.position;
    }

    public int getG_Score()
    {
        return this.g_score;
    }

    public void setG_Score(int g)
    {
        this.g_score = g;
    }

    public int getH_Score()
    {
        return this.h_estimate;
    }

    public void setH_Score(Node start, Node goal)
    {
        this.h_estimate = MobileAnimatedActor.heurisitc_cost_estimate(start, goal);
    }

    public int getF_Score()
    {
        return this.f_score;
    }

    public void setF_Score(Node start, Node goal)
    {
        this.setH_Score(start, goal);
        this.f_score = this.g_score + this.h_estimate;
    }
}
