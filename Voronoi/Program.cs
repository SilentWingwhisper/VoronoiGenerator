
using System.Diagnostics;
using System.Globalization;
using GeometryUtils;
using Point = GeometryUtils.Point;

class Program
{
    static void Main(string[] args)
    {
        int x = 0;
        while (x < 100)
        {


            List<Point> points = new List<Point>();


            double num = 100000;

            Random random = new Random();

            Stopwatch stopwatch = Stopwatch.StartNew();
            stopwatch.Start();

            double a = 500f / Math.Sqrt(num);
            double b = 500f / Math.Sqrt(num);

            for (double i = 0; i < 500; i += a)

            {
                for (double j = 0; j < 500; j += b)
                {
                    points.Add(new Point(random.NextDouble() % 500 + i, random.NextDouble() % 500 + j));
                }
            }
            List<Triangle> triangles = Delaunay.GetDelaunayTriangles(points);

            List<Polygon> polygons = VoronoiGenerator.GenerateVoronoi(triangles, new Point(), new Point(500, 500));
            stopwatch.Stop();
            Console.WriteLine($"程序运行时间: {stopwatch.ElapsedMilliseconds} 毫秒,{x}次");
            x++;
        }
    }
}


