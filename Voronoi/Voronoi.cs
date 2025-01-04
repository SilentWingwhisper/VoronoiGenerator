

using System.Collections.Generic;
using GeometryUtils;
using Point = GeometryUtils.Point;


/// <summary>
/// Voronoi图生成器，基于Delaunay三角网格，生成与给定三角形网格对应的Voronoi多边形。
/// </summary>
public static class VoronoiGenerator
{

    /// <summary>
    /// 储存图边界的最小点
    /// </summary>
    private static Point MixPoint;

    /// <summary>
    /// 储存图边界的最大点
    /// </summary>
    private static Point MaxPoint;

    /// <summary>
    /// 表示一个三角形簇，包含该簇的中心点及其所属的三角形集合。
    /// </summary>
    private class TriangularCluster
    {
        /// <summary>
        /// 簇的中心点
        /// </summary>
        public Point Center;
        /// <summary>
        /// 簇中的三角形集合
        /// </summary>
        public List<Triangle> Triangles = new List<Triangle>();
        /// <summary>
        /// 标记是否为边缘三角形簇
        /// </summary>
        public bool boolMarginalTriangle = false;
        /// <summary>
        /// 簇的起始边
        /// </summary>
        public Edge StartEdge;
        /// <summary>
        ///  簇的结束边
        /// </summary>
        public Edge EndEdge;


        public TriangularCluster(Point center)
        {
            Center = center;
        }

        /// <summary>
        /// 向簇中添加一个三角形
        /// </summary>
        /// <param name="triangle">添加的三角形</param>
        public void Add(Triangle triangle) => Triangles.Add(triangle);

        /// <summary>
        /// 向簇中添加多个三角形
        /// </summary>
        /// <param name="triangles">添加的三角形集合</param>
        public void AddRange(List<Triangle> triangles) => Triangles.AddRange(triangles);
    }

    /// <summary>
    /// 生成Voronoi图的多边形集合。
    /// </summary>
    /// <param name="originalTriangles">Delaunay三角网格的三角形集合</param>
    /// <param name="matrix">定义Voronoi图的边界的多边形</param>
    /// <returns>生成的Voronoi多边形集合</returns>
    public static List<Polygon> GenerateVoronoi(List<Triangle> originalTriangles, Point mixPoint, Point maxPoint)
    {
        MixPoint = mixPoint;
        MaxPoint = maxPoint;


        List<TriangularCluster> triangularClusters = GenerateTriangularCluster(originalTriangles);
        List<Polygon> polygons = new List<Polygon>();


        // foreach (TriangularCluster triangularCluster in triangularClusters)
        // {
        //     Polygon? polygon;
        //     if (triangularCluster.boolMarginalTriangle && triangularCluster.Triangles.Count > 0)
        //     {
        //         polygon = ProcessEdgeCluster(triangularCluster);

        //     }
        //     else polygon = ProcessCenterCluster(triangularCluster);
        //     if (polygon != null) polygons.Add((Polygon)polygon);

        // }


        return polygons;
    }

    /// <summary>
    /// 根据Delaunay三角网格生成三角形簇，每个簇由共享公共点的三角形组成。
    /// </summary>
    /// <param name="delaunayTriangles">Delaunay三角网格的三角形集合</param>
    /// <returns>包含所有三角形簇的列表</returns>
    private static List<TriangularCluster> GenerateTriangularCluster(List<Triangle> delaunayTriangles)
    {
        Dictionary<Point, HashSet<Triangle>> triangular = new Dictionary<Point, HashSet<Triangle>>();

        // 根据共享点将三角形分类
        foreach (Triangle triangle in delaunayTriangles)
        {
            foreach (Point point in triangle.Points)
            {
                if (!triangular.ContainsKey(point)) triangular[point] = new HashSet<Triangle>();
                triangular[point].Add(triangle);
            }
        }

        List<TriangularCluster> triangularClusters = new List<TriangularCluster>();

        // 为每个点生成一个三角形簇
        foreach (var tri in triangular)
        {
            List<Triangle> t = new List<Triangle>(tri.Value);
            TriangularCluster triangularCluster = SortAndGenerateCluster(new TriangularCluster(tri.Key), t);
            triangularClusters.Add(triangularCluster);
        }

        return triangularClusters;
    }

    /// <summary>
    /// 对三角形簇进行排序，并为每个簇分配中心点，判断簇是否为边缘三角形簇。
    /// </summary>
    /// <param name="cluster">当前三角形簇</param>
    /// <param name="triangles">该簇中的三角形列表</param>
    /// <returns>排序后的三角形簇</returns>
    private static TriangularCluster SortAndGenerateCluster(TriangularCluster cluster, List<Triangle> triangles)
    {
        if (triangles.Count < 2)
        {

            List<Edge> edges = triangles[0].GetRelatedVertexOfEdge(cluster.Center);
            cluster.StartEdge = edges[0];
            cluster.EndEdge = edges[1];

            cluster.boolMarginalTriangle = true;
            cluster.AddRange(triangles);
            return cluster; // 边缘三角形簇
        }
        int t = triangles.Count;

        List<Point> points = new List<Point>();
        // 收集每个三角形的点，排除簇中心点
        foreach (Triangle triangle in triangles)
        {
            foreach (Point p in triangle.Points)
            {
                if (p != cluster.Center)
                {
                    points.Add(p);

                }
            }
        }

        points.Sort();
        Point point = new Point();

        // 判断是否为边缘三角形簇并排序


        for (int i = 0; i < points.Count; i += 2)
        {

            if (points[i] != points[i + 1])
            {
                cluster.boolMarginalTriangle = true;
                point = points[i];
                break;
            }
            else if (i > points.Count - 3 && points[i] == points[i + 1])
            {

                point = points[0];
            }
        }

        cluster.StartEdge = new Edge(cluster.Center, point);

        // 根据点找到相关三角形并更新簇
        // while (triangles.Count > 0)
        {
            for (int i = 0; i < triangles.Count;)
            {
                if (triangles[i].IsPointInVertex(point))
                {
                    cluster.Add(triangles[i]);

                    foreach (Point p in triangles[i].Points)
                    {
                        if (p != cluster.Center && p != point)
                        {
                            point = p;
                            break;
                        }
                    }

                    triangles.RemoveAt(i);
                    i = 0;
                }
                else i++;
            }
        }

        cluster.EndEdge = new Edge(cluster.Center, point);
        return cluster;
    }

    /// <summary>
    /// 处理边缘三角形簇生成voronoi多边形。
    /// </summary>
    /// <param name="cluster">三角形簇</param>
    /// <returns>生成的Voronoi多边形</returns>
    private static Polygon? ProcessEdgeCluster(TriangularCluster cluster)
    {
        List<Point> points = new List<Point>();

        List<Triangle> triangles = cluster.Triangles;
        int Count = triangles.Count;

        points.AddRange(ProcessEdgeWithBoundary(triangles[0], cluster.Center, cluster.StartEdge));

        for (int i = 1; i < Count; i++)
        {
            Point circumcenter = triangles[i].GetCircumcenter();
            Point lastCircumcenter = triangles[i - 1].GetCircumcenter();
            points.AddRange(HandleCircumcenters(circumcenter, lastCircumcenter, cluster.Center));
        }

        Triangle endTriangle = triangles[Count - 1];

        List<Point> ps = ProcessEdgeWithBoundary(endTriangle, cluster.Center, cluster.EndEdge);
        ps.Reverse();
        points.AddRange(ps);

        if (Count == 1)
        {
            for (int i = 1; i < points.Count; i++)
            {
                if (points[i] == points[i - 1])
                {
                    points.RemoveAt(i);
                    i--;
                }
            }
        }

        if (points.Count == 0) return null;

        points.AddRange(AddBoundaryCorners(points[0], points[1], points[points.Count - 1], cluster.Center));
        return new Polygon(points);
    }

    /// <summary>
    /// 处理与边界相关的三角形的边缘交点。根据三角形的外接圆圆心和边界交点进行相应的处理。
    /// </summary>
    /// <param name="triangle">当前处理的三角形</param>
    /// <param name="center">Voronoi中心点</param>
    /// <param name="marginalEdge">边界边</param>
    /// <returns>返回与边界相交的点列表</returns>
    private static List<Point> ProcessEdgeWithBoundary(Triangle triangle, Point center, Edge marginalEdge)
    {

        List<Point> points = new List<Point>();

        Point circumcenter = triangle.GetCircumcenter();
        (bool? circumcenterPos, Edge edge) = triangle.IsPointInTriangle(circumcenter);
        bool? isCirInRectangle = IsInsideRectangle(circumcenter);
        Point marginalEdgeMidpoint = marginalEdge.GetMidpoint();
        //外心在三角形外
        if (circumcenterPos == false)
        {
            //外心在第一条边外并且在矩形内
            if (edge == marginalEdge && isCirInRectangle == true)
            {
                points.AddRange(MatrixIntersection(marginalEdgeMidpoint, circumcenter - marginalEdgeMidpoint));
            }
            //外心在第一条边内
            else if (edge != marginalEdge)
            {
                List<Point> ps = MatrixIntersection(circumcenter, marginalEdgeMidpoint - circumcenter);
                ps.Reverse();
                points.AddRange(ps);
            }

        }
        //外心在三角形边上并且外心就在第一条边上，此边必为直角三角形斜边
        else if (circumcenterPos == null && edge == marginalEdge)
        {

            Point p = triangle.GetUncorrelatedEdgeOfVertex(edge);
            points.AddRange(MatrixIntersection(p, circumcenter - p));
        }
        //外心在三角形内
        else points.AddRange(MatrixIntersection(circumcenter, marginalEdgeMidpoint - circumcenter));

        //外心不在矩形外
        if (isCirInRectangle != false) points.Add(circumcenter);

        return points;
    }

    /// <summary>
    /// 处理内部三角形簇的生成。
    /// </summary>
    /// <param name="cluster">三角形簇</param>
    /// <returns>生成的Voronoi多边形</returns>
    private static Polygon ProcessCenterCluster(TriangularCluster cluster)
    {
        List<Point> points = new List<Point>();

        List<Triangle> triangles = cluster.Triangles;
        int count = triangles.Count;
        points.AddRange(HandleCircumcenters(triangles[0].GetCircumcenter(), triangles[count - 1].GetCircumcenter(), cluster.Center));
        for (int i = 1; i < count; i++)
        {
            points.AddRange(HandleCircumcenters(triangles[i].GetCircumcenter(), triangles[i - 1].GetCircumcenter(), cluster.Center));
        }
        return new Polygon(points);
    }

    /// <summary>
    /// 处理两外接圆圆心之间的关系，计算它们与矩形的交点。
    /// 根据外接圆的相对位置，处理不同情况，返回交点列表。
    /// </summary>
    /// <param name="currentCircumcenter">当前外接圆圆心</param>
    /// <param name="previousCircumcenter">上一个外接圆圆心</param>
    /// <param name="voronoiCenter">Voronoi图的中心点</param>
    /// <returns>返回计算的交点列表</returns>
    private static List<Point> HandleCircumcenters(Point cir, Point lastCir, Point centre)
    {
        // 检查点是否在矩形内部
        bool? isCirInside = IsInsideRectangle(cir);
        bool? isLastCirInside = IsInsideRectangle(lastCir);

        //当前外心在矩形外
        if (isCirInside == false)
        {
            //上一个外心在矩阵内
            if (isLastCirInside == true) return MatrixIntersection(lastCir, cir - lastCir);
            //上一个外心在矩阵边上
            else if (isLastCirInside == null) return new List<Point>();
            //上一个外心在矩阵外
            else
            {
                List<Point> points = MatrixIntersection(lastCir, cir - lastCir);
                for (int i = 0; i < points.Count; i++)
                {
                    if (!Edge.BoolPointOnEdge(cir, lastCir, points[i]))
                    {
                        points.RemoveAt(i);
                        i--;
                    }
                }
                if (points.Count > 1)
                {
                    //两个外心都在矩阵外如果有交点必定为两个，points[0]和lastCir可以确定相对于centre的方向
                    points.AddRange(AddBoundaryCorners(points[0], lastCir, points[1], centre));
                }
                return points;
            }
        }
        //两个外心都在矩形边上需要确定是否含有矩形顶点
        else if (isCirInside == null && isLastCirInside == null)
        {
            //上一个外心到当前外心可以构成方向
            List<Point> points = (AddBoundaryCorners(lastCir, cir, cir, centre));
            points.Add(cir);
            return points;
        }
        else
        {
            if (isLastCirInside == false)
            {
                List<Point> points = MatrixIntersection(cir, lastCir - cir);
                points.Add(cir);
                return points;
            }
            else return new List<Point> { cir };
        }
    }


    /// <summary>
    /// 计算给定点和方向与多边形的交点。
    /// </summary>
    /// <param name="point">起始点</param>
    /// <param name="direction">方向向量</param>
    /// <param name="matrix">多边形</param>
    /// <returns>交点</returns>
    private static List<Point> MatrixIntersection(Point point, Point direction)
    {
        List<Point> points = new List<Point>();
        double t, x, y;

        // 计算交点
        
        if (direction.X != 0)
        {

            t = (MaxPoint.X - point.X) / direction.X;
            y = point.Y + t * direction.Y;
            if (t >= 0 && y <= MaxPoint.Y && y > MixPoint.Y) points.Add(new Point(MaxPoint.X, y));

            t = (MixPoint.X - point.X) / direction.X;
            y = point.Y + t * direction.Y;
            if (t >= 0 && y < MaxPoint.Y && y >= MixPoint.Y) points.Add(new Point(MixPoint.X, y));

        }
        if (direction.Y != 0)
        {

            t = (MaxPoint.Y - point.Y) / direction.Y;
            x = point.X + t * direction.X;
            if (t >= 0 && x < MaxPoint.X && x >= MixPoint.X) points.Add(new Point(x, MaxPoint.Y));

            t = (MixPoint.Y - point.Y) / direction.Y;
            x = point.X + t * direction.X;
            if (t >= 0 && x <= MaxPoint.X && x > MixPoint.X) points.Add(new Point(x, MixPoint.Y));

        }

        if (points.Count > 2) SortPointsByDistance(points, point, 0, points.Count - 1);

        return points;
    }

    /// <summary>
    /// 对一组点根据距离终点的距离进行排序。
    /// </summary>
    /// <param name="points">需要排序的点集合</param>
    /// <param name="terminus">排序参考的终点</param>
    /// <param name="low">排序的起始索引</param>
    /// <param name="high">排序的结束索引</param>
    private static void SortPointsByDistance(List<Point> points, Point terminus, int low, int high)
    {

        if (low >= high) return;

        double standard = points[low].DistanceTo(terminus);

        Point p;
        int i = low, j = high;

        while (j > i)
        {
            while (points[j].DistanceTo(terminus) >= standard && i < j) j--;
            while (points[i].DistanceTo(terminus) <= standard && i < j) i++;
            p = points[i];
            points[i] = points[j];
            points[j] = p;
        }

        p = points[j];
        points[j] = points[low];
        points[low] = p;
        SortPointsByDistance(points, terminus, low, j - 1);
        SortPointsByDistance(points, terminus, j + 1, high);

    }

    /// <summary>
    /// 判断点是否在给定的矩形范围内。
    /// </summary>
    /// <param name="point">待判断的点</param>
    /// <param name="matrix">矩形</param>
    /// <returns>若点在矩形内，返回true；若点在矩形边界上，返回null；否则返回false</returns>
    private static bool? IsInsideRectangle(Point point)
    {
        double mixX = MixPoint.X, mixY = MixPoint.Y, maxX = MaxPoint.X, maxY = MaxPoint.Y;

        if (point.X == mixX || point.X == maxX || point.Y == mixY || point.Y == maxY) return null;

        return point.X > mixX && point.X < maxX && point.Y > mixY && point.Y < maxY;
    }

    /// <summary>
    /// 根据给定的起始点、结束点和中心点，计算并添加角点（边界交点）。
    /// </summary>
    /// <param name="startP1">起始点1</param>
    /// <param name="startP2">起始点2</param>
    /// <param name="endP">结束点</param>
    /// <param name="center">Voronoi图的中心点</param>
    /// <returns>返回角点列表</returns>
    private static List<Point> AddBoundaryCorners(Point startP1, Point startP2, Point endP, Point centre)
    {
        List<Point> points = new List<Point>();
        double mixX = MixPoint.X, mixY = MixPoint.Y, maxX = MaxPoint.X, maxY = MaxPoint.Y;
        Point leftBottom = new Point(mixX, maxY), rightTop = new Point(maxY, mixY);

        if (ArePointsOnSameEdge(startP1, endP)) return points;

        bool startDirection = Edge.CrossProduct(startP1, startP2, centre) > 0;

        //起始点在左边上
        if (startP1.X == mixX)
        {
            bool mixDirection = Edge.CrossProduct(MixPoint, startP1, centre) > 0;

            //叉乘法判断左上角方向在夹角内
            if (startDirection == mixDirection)
            {
                points.Add(MixPoint);
                if (endP.X == maxX)
                {
                    points.Add(rightTop);
                }
                else if (endP.Y == maxY)
                {
                    points.Add(rightTop);
                    points.Add(MaxPoint);
                }

            }
            else
            {
                points.Add(leftBottom);

                if (endP.X == maxX)
                {
                    points.Add(MaxPoint);
                }
                else if (endP.X == mixX)
                {
                    points.Add(MaxPoint);
                    points.Add(rightTop);
                }
            }

        }
        //起始点在上边上
        if (startP1.Y == mixY)
        {
            bool mixDirection = Edge.CrossProduct(MixPoint, startP1, centre) > 0;
            //叉乘法判断左上角在夹角内
            if (startDirection == mixDirection)
            {
                points.Add(MixPoint);
                if (endP.Y == mixY)
                {
                    points.Add(leftBottom);
                }
                else if (endP.X == maxX)
                {
                    points.Add(leftBottom);
                    points.Add(MaxPoint);
                }

            }
            else
            {
                points.Add(rightTop);

                if (endP.Y == maxY)
                {
                    points.Add(MaxPoint);
                }
                else if (endP.X == mixX)
                {
                    points.Add(MaxPoint);
                    points.Add(rightTop);
                }
            }

        }
        //起始点在右边上
        if (startP1.X == maxX)
        {
            bool maxDirection = Edge.CrossProduct(MaxPoint, startP1, centre) > 0;
//叉乘法判断右下角在夹角内
            if (startDirection == maxDirection)
            {
                points.Add(MaxPoint);
                if (endP.X == mixX) points.Add(leftBottom);
                else if (endP.Y == mixY)
                {
                    points.Add(leftBottom);
                    points.Add(MixPoint);
                }
            }
            else
            {
                points.Add(rightTop);
                if (endP.X == mixX) points.Add(MixPoint);
                else if (endP.Y == maxY)
                {
                    points.Add(MixPoint);
                    points.Add(leftBottom);
                }
            }
        }
        //起始点在下边上
        if (startP1.Y == maxY)
        {
            bool maxDirection = Edge.CrossProduct(MaxPoint, startP1, centre) > 0;
            //叉乘法判断右下角在夹角内
            if (startDirection == maxDirection)
            {
                points.Add(MaxPoint);
                if (endP.Y == mixY) points.Add(rightTop);
                else if (endP.X == mixX)
                {
                    points.Add(rightTop);
                    points.Add(MixPoint);
                }
            }
            else
            {
                points.Add(leftBottom);

                if (endP.Y == mixY) points.Add(MixPoint);
                if (endP.X == maxX)
                {
                    points.Add(MixPoint);
                    points.Add(rightTop);
                }
            }

        }
        return points;
    }


    /// <summary>
    /// 检查两个点是否位于同一边界线上（即，两个点是否在同一条边上）。
    /// </summary>
    /// <param name="p1">点1</param>
    /// <param name="p2">点2</param>
    /// <returns>如果两个点在同一条边上，则返回true，否则返回false</returns>
    private static bool ArePointsOnSameEdge(Point p1, Point p2)
    {
        double mixX = MixPoint.X, mixY = MixPoint.Y, maxX = MaxPoint.X, maxY = MaxPoint.Y;

        if ((p1.X == mixX && p1.X == p2.X)
        || (p1.X == maxX && p1.X == p2.X)
        || (p1.Y == mixY && p1.Y == p2.Y)
        || (p1.Y == maxY && p1.Y == p2.Y)) return true;
        return false;
    }
}


