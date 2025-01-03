
using System.Diagnostics;
using System.Globalization;
using GeometryUtils;
using Point = GeometryUtils.Point;

class Program
{
    static void Main(string[] args)
    {
        while (true)
        {


            List<Point> points = new List<Point>();

            Console.WriteLine("请输入生成列数");
            float columns = int.Parse(Console.ReadLine());
            Console.WriteLine("请输入生成排数");

            float row = int.Parse(Console.ReadLine());

            Random random = new Random();

            Stopwatch stopwatch = Stopwatch.StartNew();
            stopwatch.Start();

            float a = 500f / columns;
            float b = 500f / row;

            for (float i = 0; i < 500; i += a)

            {
                for (float j = 0; j < 500; j += b)
                {
                    points.Add(new Point((float)random.NextDouble() % 500 + i, (float)random.NextDouble() % 500 + j));
                }
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


