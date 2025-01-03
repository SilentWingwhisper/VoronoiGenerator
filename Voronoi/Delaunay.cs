using GeometryUtils;
using System.Collections.Generic;

namespace GeometryUtils
{
	/// <summary>
	/// 提供实现Delaunay三角剖分的静态方法。
	/// </summary>
	public static class Delaunay
	{
		/// <summary>
		/// 用于表示一个三角形的外接圆。
		/// </summary>
		private class Circumcircle
		{
			private Point Circumcenter;
			private float Radius;

			/// <summary>
			/// 创建一个新的外接圆对象，基于指定的三角形。
			/// </summary>
			/// <param name="triangle">需要计算外接圆的三角形。</param>
			public Circumcircle(Triangle triangle)
			{
				Circumcenter = triangle.GetCircumcenter();
				Radius = triangle.Points[0].DistanceTo(Circumcenter);
			}

			/// <summary>
			/// 获取外接圆的圆心。
			/// </summary>
			/// <returns>外接圆的圆心。</returns>
			public Point GetCircumcenter() => Circumcenter;

			/// <summary>
			/// 获取外接圆的半径。
			/// </summary>
			/// <returns>外接圆的半径。</returns>
			public float GetRadius() => Radius;
		}

		/// <summary>
		/// 通过指定的点集生成Delaunay三角形。
		/// </summary>
		/// <param name="points">用于生成三角形的点集。</param>
		/// <returns>包含Delaunay三角形的列表。</returns>
		public static List<Triangle> GetDelaunayTriangles(List<Point> points)
		{
			List<Triangle> delaunayTriangles = new List<Triangle>();
			points.Sort();

			// 临时储存的三角形
			Dictionary<Triangle, Circumcircle> temporaryTriangles = new Dictionary<Triangle, Circumcircle>();

			// 生成初始三角形
			Triangle initialTriangle = GenerateSuperTriangle(points);
			temporaryTriangles.Add(initialTriangle, new Circumcircle(initialTriangle));

			foreach (Point point in points)
			{
				List<Edge> temporaryEdge = new List<Edge>();
				List<Triangle> deletionTriangle = new List<Triangle>();

				foreach (KeyValuePair<Triangle, Circumcircle> temporaryTriangle in temporaryTriangles)
				{
					// 判断点是否在外接圆内
					if (IsPointInsideCircumcircle(point, temporaryTriangle.Value))
					{
						deletionTriangle.Add(temporaryTriangle.Key);
						Edge[] edges = temporaryTriangle.Key.GetEdges();

						if (temporaryEdge.Count > 0)
							ReshapedEdge(edges, temporaryEdge);
						else
							temporaryEdge.AddRange(edges);
					}
					// 判断点是否在外接圆右侧
					else if (JudgePointRightCircumcircle(point, temporaryTriangle.Value))
					{
						delaunayTriangles.Add(temporaryTriangle.Key);
						deletionTriangle.Add(temporaryTriangle.Key);
					}
				}

				ReshapedTriangle(point, temporaryTriangles, temporaryEdge, deletionTriangle);
			}

			delaunayTriangles.AddRange(temporaryTriangles.Keys);
			DeleteSuperTriangle(delaunayTriangles, initialTriangle);

			return delaunayTriangles;
		}


		/// <summary>
		/// 生成一个包含所有点的超级三角形,用于初始化Delaunay三角剖分。
		/// </summary>
		private static Triangle GenerateSuperTriangle(List<Point> points)
		{
			float minY = points[0].Y;
			float minX = points[0].X;
			float maxY = 0;
			float maxX = points[points.Count - 1].X;

			foreach (Point point in points)
			{
				if (point.Y < minY) minY = point.Y;
				else if (point.Y > maxY) maxY = point.Y;
			}

			float height = maxY - minY;
			float width = maxX - minX;

			Point a = new Point(minX - height, minY);
			Point b = new Point(maxX + height, minY);
			Point c = new Point(minX + width / 2, maxY + width / 2);

			return new Triangle(a, b, c);
		}

		/// <summary>
		/// 移除所有包含超级三角形顶点的三角形。
		/// </summary>
		private static void DeleteSuperTriangle(List<Triangle> delaunayTriangles, Triangle initialTriangle)
		{
			List<int> deletionTriangleIndex = new List<int>();

			for (int i = 0; i < delaunayTriangles.Count; i++)
			{
				foreach (Point point in initialTriangle.Points)
				{
					if (delaunayTriangles[i].IsPointInVertex(point))
					{
						deletionTriangleIndex.Add(i);
						break;
					}
				}
			}
			deletionTriangleIndex.Sort();
			deletionTriangleIndex.Reverse();
			foreach (int index in deletionTriangleIndex)
			{
				delaunayTriangles.RemoveAt(index);
			}
		}

		/// <summary>
		/// 更新边集合，移除重复的边。
		/// </summary>
		private static void ReshapedEdge(Edge[] edges, List<Edge> temporaryEdge)
		{
			for (int i = 0; i < edges.Length; i++)
			{
				for (int j = 0; j < temporaryEdge.Count; j++)
				{
					if (edges[i] == temporaryEdge[j])
					{
						temporaryEdge.RemoveAt(j);
						break;
					}
					if (j == temporaryEdge.Count - 1)
					{
						temporaryEdge.Add(edges[i]);
						break;
					}
				}
			}
		}

		/// <summary>
		/// 根据新插入的点更新三角形集合，删除不再有效的三角形，并添加新的三角形。
		/// </summary>
		private static void ReshapedTriangle(Point point, Dictionary<Triangle, Circumcircle> temporaryTriangles, List<Edge> temporaryEdge, List<Triangle> deletionTriangle)
		{
			foreach (Edge edge in temporaryEdge)
			{
				Triangle temporaryTriangle = new Triangle(point, edge);
				temporaryTriangles.Add(temporaryTriangle, new Circumcircle(temporaryTriangle));
			}
			foreach (Triangle triangle in deletionTriangle)
			{
				temporaryTriangles.Remove(triangle);
			}
		}

		/// <summary>
		/// 判断点是否在外接圆内。
		/// </summary>
		private static bool IsPointInsideCircumcircle(Point point, Circumcircle circumcircle)
		{
			return point.DistanceTo(circumcircle.GetCircumcenter()) < circumcircle.GetRadius();
		}

		/// <summary>
		/// 判断点是否位于外接圆右侧。
		/// </summary>
		private static bool JudgePointRightCircumcircle(Point point, Circumcircle circumcircle)
		{
			return circumcircle.GetCircumcenter().X + circumcircle.GetRadius() < point.X;
		}
	}
}