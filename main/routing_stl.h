namespace Map_data {
  
  struct Point_coordinate {
    double x, y;
  };

  Point_coordinate points[] = {
    {116.328622,40.011544}, // #0 紫荆园
    {116.328675,40.010858}, // #1
    {116.32816,40.010821}, // #2
    {116.327195,40.009942}, // #3
    {116.327254,40.009432}, // #4
    {116.327527,40.006211}, // #5
    {116.327581,40.005652}, // #6 清芬园
    {116.327672,40.004641}, // #7
    {116.328353,40.004666}, // #8 文图
    {116.32779,40.003331}, // #9
    {116.327881,40.00227}, // #10
    {116.328337,40.002283}, // #11 三教
    {116.330644,40.002369}, // #12 六教
    {116.327404,40.002233}, // #13 四教
    {116.326417,40.002201}, // #14 五教
    {116.326261,40.002279}, // #15
    {116.32492,40.002172}, // #16 清华学堂
    {116.32491,40.002044}, // #17
    {116.324389,40.001999}, // #18
    {116.324395,40.00149}, // #19 一教
    {116.327959,40.001022}, // #20
    {116.328936,40.001039}, // #21 新清华学堂
    {116.33069,40.001076}, // #22
    {116.332535,40.001137}, // #23 主楼
    {116.330593,40.003615}, // #24
    {116.33238,40.003681}, // #25 综体
    {116.330502,40.00477}, // #26
    {116.325347,40.000993}, // #27
    {116.324397,40.000883}, // #28 二校门
    {116.324191,40.001987}, // #29
    {116.324035,40.003293}, // #30
    {116.324414,40.003346}, // #31
    {116.324811,40.003325}, // #32
    {116.324709,40.004303}, // #33
    {116.324687,40.00471}, // #34 老馆
    {116.32296,40.004283}, // #35
    {116.322845,40.005952}, // #36
    {116.323872,40.006002}, // #37 北馆
    {116.324751,40.003936}, // #38
    {116.326221,40.003994}, // #39
    {116.326682,40.004207}, // #40
    {116.326956,40.004585}, // #41
    {116.330392,40.006659}, // #42
    {116.327887,40.006318}, // #43
    {116.328246,40.006556}, // #44
    {116.330311,40.009095}, // #45
    {116.328815,40.009009}, // #46
    {116.328471,40.008993}, // #47
    {116.327409,40.009428}, // #48
    {116.328756,40.009929}, // #49 C楼
  };

  struct Road_index {
    int u, v;
  };

  Road_index Roads[] = {
    {0, 1}, // 
    {1, 2}, // 
    {2, 3}, // 
    {3, 4}, // 学堂路
    {4, 5}, // 学堂路
    {5, 6}, // 学堂路
    {6, 7}, // 学堂路
    {7, 9}, // 学堂路
    {9, 10}, // 学堂路
    {7, 8}, // 
    {10, 11}, // 
    {11, 12}, // 
    {10, 13}, // 
    {13, 14}, // 
    {14, 15}, // 
    {15, 16}, // 
    {16, 17}, // 
    {17, 18}, // 
    {18, 19}, // 
    {10, 20}, // 学堂路
    {20, 21}, // 清华路
    {21, 22}, // 清华路
    {22, 23}, // 清华路
    {12, 22}, // 新民路
    {12, 24}, // 新民路
    {24, 25}, // 
    {8, 26}, // 
    {24, 26}, // 
    {20, 27}, // 清华路
    {27, 28}, // 清华路
    {19, 28}, // 
    {18, 29}, // 
    {29, 30}, // 
    {30, 31}, // 
    {31, 32}, // 
    {16, 32}, // 
    {32, 38}, // 
    {38, 33}, //
    {33, 34}, // 
    {33, 35}, // 
    {35, 36}, // 
    {36, 37}, // 至善路
    {5, 37}, // 至善路
    {38, 39}, // 
    {39, 40}, // 
    {40, 41}, // 
    {41, 7}, // 
    {26, 42}, // 新民路
    {5, 43}, // 至善路
    {43, 44}, // 至善路
    {44, 42}, // 至善路
    {42, 45}, // 新民路
    {45, 46}, // 紫荆路
    {46, 47}, // 紫荆路
    {47, 48}, // 紫荆路
    {48, 4}, // 紫荆路
    {46, 49}, // 
    {49, 1}, // 
  };

}
#include <StandardCplusplus.h>
#include <algorithm>
#include <vector>
#include <queue>
#include <math.h>

namespace Route {

  const double PI = acos(-1);
	const int Point_Number = sizeof(Map_data::points) / sizeof(Map_data::Point_coordinate);
	const int Road_Number = sizeof(Map_data::Roads) / sizeof(Map_data::Road_index);

	bool inq[Point_Number];
	int path[Point_Number];
	double dis[Point_Number];
	std::queue <int> que;
	std::vector <int> routes;
	std::vector <int> directions;

	bool check(double &a, double b);

	double sqr(double x);

	double calculate_dist(int s, int t);

	double calculate_dist(double x, double y, int p);

	void route_by_id(int s, int t);

	bool connected(int u, int v);

	int find_nearest_point(double x, double y);

	int find_nearest_road(double x, double y);

	void route(double x, double y, int t);

	int calc_direction(int a, int b, int c);

	int calc_direction(double x, double y, int b, int c);

  int adjust_direction(double x_now, double y_now, double x_nxt, double y_nxt, double direction_now);

}

bool Route::check(double &a, double b) {
	if (a < 0 || b < a) return a = b, true;
	return false;
}

double Route::sqr(double x) {
	return x * x;
}

double Route::calculate_dist(int s, int t) {
	// reference coordinate 116.326836,40.00366
	double dx = 85.1752429951, dy = 111.1949266445;
	double disx = (Map_data::points[s].x - Map_data::points[t].x) * dx;
	double disy = (Map_data::points[s].y - Map_data::points[t].y) * dy;
	return sqrt( sqr(disx) + sqr(disy) );
}

double Route::calculate_dist(double x, double y, int t) {
	// reference coordinate 116.326836,40.00366
	double dx = 85.1752429951, dy = 111.1949266445;
	double disx = (x - Map_data::points[t].x) * dx;
	double disy = (y - Map_data::points[t].y) * dy;
	return sqrt( sqr(disx) + sqr(disy) );
}


bool Route::connected(int u, int v) {
  for (short i = 0; i < Route::Road_Number; i++) {
    if (Map_data::Roads[i].u == u && Map_data::Roads[i].v == v) return true;
    if (Map_data::Roads[i].v == u && Map_data::Roads[i].u == v) return true;
  }
  return false;
}

void Route::route_by_id(int s, int t) {
	while (!que.empty()) que.pop();
	for (int i = 0; i < Point_Number; i++) {
		inq[i] = false;
		dis[i] = -1;
	}
	dis[s] = 0;
	inq[s] = true;
	que.push(s);
	while (!que.empty()) {
		int x = que.front();
		que.pop();
		inq[x] = false;
		for (int to = 0; to < Point_Number; to++) {
			if (!connected(x, to)) continue;
			if (check(dis[to], dis[x] + calculate_dist(x, to))) {
				if (!inq[to]) {
					que.push(to);
					inq[to] = true;
				}
				path[to] = x;
			}
		}
	}
	routes.clear();
	routes.push_back(t);
	while (t != s) {
		t = path[t];
		routes.push_back(t);
	}
	std::reverse(routes.begin(), routes.end());
}

int Route::find_nearest_point(double x, double y) {
	double mindis = -1;
	int id;
	for (int i = 0; i < Point_Number; i++) {
		double d = calculate_dist(x, y, i);
		if (check(mindis, d)) id = i;
	}
	if (mindis < 0.05) return id;
	else return -1;
}

int Route::find_nearest_road(double x, double y) {
	double mindis = -1;
	int id;
	for (int i = 0; i < Road_Number; i++) {
		int u = Map_data::Roads[i].u, v = Map_data::Roads[i].v;
		double a = calculate_dist(x, y, u);
		double b = calculate_dist(x, y, v);
		double c = calculate_dist(u, v);
		double p = (a + b + c) / 2;
		double area = sqrt(p * (p - a) * (p - b) * (p - c));
		double h = 2 * area / c;
		if (check(mindis, h)) id = i;
	}
	return id;
}

int Route::calc_direction(int a, int b, int c) {
	double x1 = Map_data::points[b].x - Map_data::points[a].x;
	double y1 = Map_data::points[b].y - Map_data::points[a].y;
	double x2 = Map_data::points[c].x - Map_data::points[b].x;
	double y2 = Map_data::points[c].y - Map_data::points[b].y;
	// (x1, y1), (x2, y2)
	double cos_theta = (x1 * x2 + y1 * y2) / (sqrt(sqr(x1) + sqr(y1)) * sqrt(sqr(x2) + sqr(y2)));
	if (cos_theta > 0.7) return 0;
	if (x1 * y2 - y1 * x2 > 0) return 1; // Left
	else return -1; // Right
}

int Route::calc_direction(double x, double y, int b, int c) {
	double x1 = Map_data::points[b].x - x;
	double y1 = Map_data::points[b].y - y;
	double x2 = Map_data::points[c].x - Map_data::points[b].x;
	double y2 = Map_data::points[c].y - Map_data::points[b].y;
	// (x1, y1), (x2, y2)
	double cos_theta = (x1 * x2 + y1 * y2) / (sqrt(sqr(x1) + sqr(y1)) * sqrt(sqr(x2) + sqr(y2)));
	if (cos_theta > 0.7) return 0;
	if (x1 * y2 - y1 * x2 > 0) return 1; // Left
	else return -1; // Right
}

void Route::route(double x, double y, int t) {
	int id = find_nearest_point(x, y);
	if (id == -1) {
		id = find_nearest_road(x, y);
		int u = Map_data::Roads[id].u, v = Map_data::Roads[id].v;
		route_by_id(u, t);
		double disu = dis[t] + calculate_dist(x, y, u);
		route_by_id(v, t);
		double disv = dis[t] + calculate_dist(x, y, v);
		id = disu < disv ? u : v;
		route_by_id(id, t);
	} else {
		route_by_id(id, t);
		std::reverse(routes.begin(), routes.end());
		routes.pop_back();
		std::reverse(routes.begin(), routes.end());
	}
	directions.clear();
	if ((int) routes.size() > 1) directions.push_back(calc_direction(x, y, routes[0], routes[1]));
	for (int i = 1; i < (int) routes.size() - 1; i++)
		directions.push_back(calc_direction(routes[i - 1], routes[i], routes[i + 1]));
}

int Route::adjust_direction(double x_now, double y_now, double x_nxt, double y_nxt, double direction_now) {
  // return :
  // 0 : no need to adjust
  // 1 : turn left
  // -1 : turn right
  double vector_x = x_nxt - x_now, vector_y = y_nxt - y_now;
  double direction_nxt = atan2(vector_x, vector_y);
  if ((fabs(direction_now - direction_nxt) < PI / 6) ||
    (fabs(direction_now + 2 * PI - direction_nxt) < PI / 6) ||
    (fabs(direction_now - 2 * PI - direction_nxt) < PI / 6)
  ) return 0; // angle < 30 degree : no need to adjust
  if (direction_now < 0) {
    if ((direction_now < direction_nxt) && (direction_nxt < direction_now + PI)) return 1; // turn left
    else return -1; // turn right
  } else {
    if ((direction_nxt < direction_now) && (direction_now - PI < direction_nxt) return -1; // turn right
    else return 1; // turn left
  }
}