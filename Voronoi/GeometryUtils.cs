using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata.Ecma335;

/// <summary>
/// 提供几何结构体和基本操作，包括点(Point)、边(Edge)以及相关几何计算功能。
/// </summary>
namespace GeometryUtils
{
    /// <summary>
    /// 表示一个二维平面上的点。
    /// 提供基本的点运算，如加减、乘除、比较等。
    /// </summary>
    public struct Point : IComparable<Point>
    {
        public float X; // 点的X坐标
        public float Y; // 点的Y坐标

        /// <summary>
        /// 使用指定的坐标初始化点。
        /// </summary>
        public Point(float x, float y)
        {
            X = x;
            Y = y;
        }

        /// <summary>
        /// 初始化默认点 (0, 0)。
        /// </summary>
        public Point()
        {
            X = 0;
            Y = 0;
        }

        public Point(Point point)
        {
            this = new(point.X, point.Y);
        }
        // 重载运算符以支持点的比较和运算
        public static bool operator ==(Point left, Point right) => left.X == right.X && left.Y == right.Y;
        public static bool operator !=(Point left, Point right) => !(left == right);
        public static bool operator >(Point left, Point right) => left.X > right.X || (left.X == right.X && left.Y > right.Y);
        public static bool operator <(Point left, Point right) => left.X < right.X || (left.X == right.X && left.Y < right.Y);

        public override bool Equals(object obj) => obj is Point other && this == other;
        public override int GetHashCode() => HashCode.Combine(X, Y);

        public static Point operator +(Point left, Point right) => new Point(left.X + right.X, left.Y + right.Y);
        public static Point operator -(Point left, Point right) => new Point(left.X - right.X, left.Y - right.Y);
        public static Point operator *(Point left, Point right) => new Point(left.X * right.X, left.Y * right.Y);
        public static Point operator *(Point point, float multiplier) => new Point(point.X * multiplier, point.Y * multiplier);
        public static Point operator /(Point left, Point right) => new Point(left.X / right.X, left.Y / right.Y);
        public static Point operator /(Point point, float divisor) => new Point(point.X / divisor, point.Y / divisor);


        /// <summary>
        /// 计算当前点与指定点之间的欧几里得距离.
        /// </summary>
        /// <param name="point">目标点。</param>
        /// <returns>当前点到目标点的距离。</returns>
        public float DistanceTo(Point point) => MathF.Sqrt((X - point.X) * (X - point.X) + (Y - point.Y) * (Y - point.Y));



        int IComparable<Point>.CompareTo(Point other)
        {
            // 首先比较 X 坐标
            int xComparison = X.CompareTo(other.X);

            if (xComparison != 0)
            {
                return xComparison;
            }

            // 如果 X 坐标相等，比较 Y 坐标
            return Y.CompareTo(other.Y);
        }
    }

    /// <summary>
    /// 表示一条由两个点定义的线段。
    /// 提供线段的中点、法向量计算以及线段交点计算等功能。
    /// </summary>
    public struct Edge
    {
        public Point Start; // 线段起点
        public Point End;   // 线段终点

        /// <summary>
        /// 使用指定的起点和终点初始化线段。
        /// </summary>
        public Edge(Point start, Point end)
        {
            Start = start;
            End = end;
        }

        /// <summary>
        /// 使用另一个线段初始化线段。
        /// </summary>
        public Edge(Edge edge)
        {
            Start = edge.Start;
            End = edge.End;
        }

        // 重载运算符以支持线段的比较
        public static bool operator ==(Edge left, Edge right) =>
            (left.Start == right.Start && left.End == right.End) ||
            (left.Start == right.End && left.End == right.Start);

        public static bool operator !=(Edge left, Edge right) => !(left == right);
        public override bool Equals(object obj) => obj is Edge other && this == other;
        public override int GetHashCode() => HashCode.Combine(Start.X, End.Y);

        /// <summary>
        /// 计算当前线段与另一条线段的交点。
        /// </summary>
        public Point? IntersectionTo(Edge edge) => IntersectionTo(edge.Start, edge.End);

        /// <summary>
        /// 计算当前线段与由两个点定义的线段的交点。
        /// </summary>
        public Point? IntersectionTo(Point start, Point end) => IntersectionTo(this, new Edge(start, end));

        /// <summary>
        /// 计算两条线段的交点。
        /// 如果线段不相交或平行，则返回 null。
        /// </summary>
        private Point? IntersectionTo(Edge A, Edge B)
        {
            Point a1 = A.Start;
            Point a2 = A.End;
            Point b1 = B.Start;
            Point b2 = B.End;

            // 计算法向量
            Point nA = GetNormalVector(A.Start, A.End);
            Point nB = GetNormalVector(B.Start, B.End);

            // 计算投影距离
            float b1nA = b1.X * nA.X + b1.Y * nA.Y;
            float b2nA = b2.X * nA.X + b2.Y * nA.Y;
            float anA = a1.X * nA.X + a1.Y * nA.Y;

            float a1nB = a1.X * nB.X + a1.Y * nB.Y;
            float a2nB = a2.X * nB.X + a2.Y * nB.Y;
            float bnB = b1.X * nB.X + b1.Y * nB.Y;

            // 检查线段是否不相交
            if ((b1nA - anA) * (b2nA - anA) >= 0 || (a1nB - bnB) * (a2nB - bnB) >= 0)
            {
                return null;
            }

            // 计算交点
            float denominator = nA.X * nB.Y - nA.Y * nB.X;

            float fraction = (a1nB - bnB) / denominator;

            float x = a1.X + fraction * nA.Y;
            float y = a1.Y - fraction * nA.X;

            return new Point(x, y);
        }

        /// <summary>
        /// 获取线段的中点。
        /// </summary>
        public Point GetMidpoint() => GetMidpoint(Start, End);

        /// <summary>
        /// 获取两个点的中点。
        /// </summary>
        public static Point GetMidpoint(Point point1, Point point2) => (point1 + point2) / 2;

        /// <summary>
        /// 获取线段的法向量。
        /// </summary>
        public Point GetNormalVector() => GetNormalVector(Start, End);

        /// <summary>
        /// 计算两个点定义的线段的法向量。
        /// </summary>
        public static Point GetNormalVector(Point start, Point end) => new Point(end.Y - start.Y, start.X - end.X);

        /// <summary>
        /// 通过点的起始和结束点来判断点是否在线段上。
        /// </summary>
        /// <param name="point">需要判断的点。</param>
        /// <returns>如果点在线段上，返回 true；否则，返回 false。</returns>
        public bool BoolPointOnEdge(Point point) => BoolPointOnEdge(Start, End, point);

        /// <summary>
        /// 判断一个点是否在线段上。
        /// </summary>
        /// <param name="start">线段的起始点。</param>
        /// <param name="end">线段的结束点。</param>
        /// <param name="point">需要判断的点。</param>
        /// <returns>如果点在线段上，返回 true；否则，返回 false。</returns>
        public static bool BoolPointOnEdge(Point start, Point end, Point point)
        {
            // 判断三个点是否共线
            bool isCollinear = CrossProduct(start, end, point) == 0;

            // 进一步检查点是否在线段范围内
            if (isCollinear)
            {
                return point.X >= Math.Min(start.X, end.X) && point.X <= Math.Max(start.X, end.X) &&
                       point.Y >= Math.Min(start.Y, end.Y) && point.Y <= Math.Max(start.Y, end.Y);
            }

            return false;
        }
        public float CrossProduct(Point point) => CrossProduct(Start, End, point);
        public static float CrossProduct(Point start, Point end, Point point) => (end.X - start.X) * (point.Y - start.Y) - (end.Y - start.Y) * (point.X - start.X);
    }


    /// <summary>
    /// 表示一个二维平面上的三角形，由三个顶点和三条边构成。
    /// </summary>
    public struct Triangle
    {
        /// <summary>
        /// 三角形的三个顶点。
        /// </summary>
        public Point[] Points;

        /// <summary>
        /// 三角形的三条边。
        /// </summary>
        public Edge[] Edges;

        /// <summary>
        /// 使用三个顶点初始化三角形。
        /// </summary>
        /// <param name="a">顶点A</param>
        /// <param name="b">顶点B</param>
        /// <param name="c">顶点C</param>
        public Triangle(Point a, Point b, Point c)
        {
            Points = new Point[3] { a, b, c };
            Edges = new Edge[3] { new Edge(a, b), new Edge(b, c), new Edge(c, a) };
        }

        /// <summary>
        /// 使用一个顶点和一条边初始化三角形。
        /// </summary>
        /// <param name="a">顶点A</param>
        /// <param name="bc">边BC</param>
        public Triangle(Point a, Edge bc)
        {
            Points = new Point[3] { a, bc.Start, bc.End };
            Edges = new Edge[3] { new Edge(a, bc.Start), bc, new Edge(bc.End, a) };
        }

        public static bool operator ==(Triangle left, Triangle right)
        {
            foreach (Point point in left.Points)
            {
                if (!right.IsPointInVertex(point)) return false;

            }
            return true;
        }

        public static bool operator !=(Triangle left, Triangle right)
        {
            return !(left == right);
        }

        /// <summary>
        /// 检查给定点是否是三角形的顶点。
        /// </summary>
        /// <param name="point">待检查的点</param>
        /// <returns>如果是顶点，返回true；否则返回false。</returns>
        public bool IsPointInVertex(Point point)
        {
            for (int i = 0; i < Points.Length; i++)
            {
                if (Points[i] == point)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 检查是否为三角形的边
        /// </summary>
        /// <param name="edge">待检测的边</param>
        /// <returns>如果是三角形的边，则返回true，否则false。</returns>
        public bool IsTriangularEdge(Edge edge)
        {
            for (int i = 0; i < Points.Length; i++)
            {
                if (Edges[i] == edge)
                {
                    return true;
                }
            }
            return false;
        }

        /// <summary>
        /// 计算三角形的外心（外接圆圆心）。
        /// </summary>
        /// <returns>外心的坐标。</returns>
        public Point GetCircumcenter()
        {
            float ax = Points[0].X, ay = Points[0].Y;
            float bx = Points[1].X, by = Points[1].Y;
            float cx = Points[2].X, cy = Points[2].Y;
            float d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
            float ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
            float uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;

            return new Point(ux, uy);
        }

        /// <summary>
        /// 计算三角形的内心（内切圆圆心）。
        /// </summary>
        /// <returns>内心的坐标。</returns>
        public Point GetIncenter()
        {
            Point A = Points[0];
            Point B = Points[1];
            Point C = Points[2];

            float a = B.DistanceTo(C);
            float b = C.DistanceTo(A);
            float c = A.DistanceTo(B);

            float x = (a * A.X + b * B.X + c * C.X) / (a + b + c);
            float y = (a * A.Y + b * B.Y + c * C.Y) / (a + b + c);
            return new Point(x, y);
        }

        /// <summary>
        /// 获取三角形的顶点数组。
        /// </summary>
        /// <returns>顶点数组。</returns>
        public Point[] GetPoints()
        {
            return Points;
        }

        /// <summary>
        /// 获取三角形的边数组。
        /// </summary>
        /// <returns>边数组。</returns>
        public Edge[] GetEdges()
        {
            return Edges;
        }

        /// <summary>
        /// 获取给定点与三角形中其他顶点构成的边。
        /// </summary>
        /// <param name="point">给定的点</param>
        /// <returns>与该点相关的边集合。</returns>
        public List<Edge> GetRelatedVertexOfEdge(Point point)
        {
            List<Edge> edges = new List<Edge>();
            foreach (Point vertex in Points)
            {
                if (point != vertex)
                {
                    edges.Add(new Edge(point, vertex));
                }
            }
            return edges;
        }

        public Point GetUncorrelatedEdgeOfVertex(Edge edge)
        {
            Point point = new Point();
            foreach (Point vertex in Points)
            {
                if (vertex != edge.Start && vertex != edge.End) point = vertex;
            }
            return point;
        }

        /// <summary>
        /// 使用同侧法判断给定点是否在三角形内部。
        /// </summary>
        /// <param name="point">待判断的点</param>
        /// <returns>如果点与边重合返回null，否则返回是否在三角形内的布尔值。</returns>
        public (bool? isInside, Edge edge) IsPointInTriangle(Point point)
        {
            Point a = Points[0];
            Point b = Points[1];
            Point c = Points[2];

            float cross1 = (b.X - a.X) * (point.Y - a.Y) - (b.Y - a.Y) * (point.X - a.X);
            float cross2 = (c.X - b.X) * (point.Y - b.Y) - (c.Y - b.Y) * (point.X - b.X);
            float cross3 = (a.X - c.X) * (point.Y - c.Y) - (a.Y - c.Y) * (point.X - c.X);

            if (cross1 == 0 || cross2 == 0 || cross3 == 0)
            {
                return (null, cross1 == 0 ? new Edge(a, b) : cross2 == 0 ? new Edge(b, c) : new Edge(c, a));
            }

            bool sameSide1 = cross1 > 0 && cross2 > 0 && cross3 > 0;
            bool sameSide2 = cross1 < 0 && cross2 < 0 && cross3 < 0;
            if (sameSide1 || sameSide2)
            {
                return (true, new Edge());
            }
            else
            {
                Edge ab = new Edge(a, b);
                float abp = point.DistanceTo(ab.GetMidpoint());
                Edge bc = new Edge(b, c);
                float bcp = point.DistanceTo(bc.GetMidpoint());
                Edge ca = new Edge(c, a);
                float cap = point.DistanceTo(ca.GetMidpoint());

                if (abp < bcp && abp < cap) return (false, ab);
                else if (bcp < abp && bcp < cap) return (false, bc);
                else return (false, ca);
            }


        }

    }

    public struct Polygon
    {
        public Point[] Points;
        public Edge[] Edges;

        public Polygon(Point[] points)
        {
            Points = points;
            Edges = new Edge[points.Length];
            for (int i = 0; i < points.Length; i++)
            {
                if (i == 0) Edges[i] = new Edge(points[points.Length - 1], points[i]);
                else Edges[i] = new Edge(points[i], points[i - 1]);
            }
        }

        public Polygon(List<Point> points)
        {
            Edges = new Edge[points.Count];
            Points = new Point[points.Count];
            for (int i = 0; i < points.Count; i++)
            {
                Points[i] = points[i];
                if (i == 0) Edges[i] = new Edge(points[points.Count - 1], points[i]);
                else Edges[i] = new Edge(points[i], points[i - 1]);
            }

        }

        public Point GetCentroid()
        {
            
            float A = 0;//面积
            Point Centroid = new Point() ;

            int count = Points.Length;
            for (int i = 0;i<count;i++)
            {
                Point p1 = Points[i];
                Point p2 = Points[(i+1)%count];
                float CrossProduct = p1.X*p2.Y-p2.X*p1.Y;
                A+=CrossProduct;
                Centroid.X+=(p1.X+p2.X)*CrossProduct;
                Centroid.Y+=(p1.Y+p2.Y)*CrossProduct;
            }
            A/=2;
            Centroid.X/=(6*A);
            Centroid.Y/=(6*A);
            return Centroid;
        }
    }
}