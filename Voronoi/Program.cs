
using System.Diagnostics;
using GeometryUtils;
using Point = GeometryUtils.Point;

class Program
{
    static void Main(string[] args)
    {
        while (true)
        {


            List<Point> points = new List<Point>();

            Console.WriteLine("请输入生成数量");
            string? input = Console.ReadLine();

            Random random = new Random();
            if (input != null)
            {
                Stopwatch stopwatch = Stopwatch.StartNew();
                stopwatch.Start();
                int num = int.Parse(input);
                for (int i = 0; i < num; i++)
                {
                    points.Add(new Point((float)random.NextDouble() % 1000, (float)random.NextDouble() % 1000));
                }
                List<Triangle> triangles = Delaunay.GetDelaunayTriangles(points);
                List<Point> points1 = new List<Point>() { new Point(0, 0), new Point(0, 500), new Point(500, 0), new Point(500, 500) };
                Polygon polygon = new Polygon(points1);
               List<Polygon> polygons = VoronoiGenerator.GenerateVoronoi(triangles, polygon);
                stopwatch.Stop();
                Console.WriteLine($"程序运行时间: {stopwatch.ElapsedMilliseconds} 毫秒");
            }
        }
    }
}


